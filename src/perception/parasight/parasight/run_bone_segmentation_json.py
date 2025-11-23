#!/usr/bin/env python3
"""Bone segmentation feature extraction using DINOv3."""

import argparse
import os
import time
import pickle

import numpy as np
import torch
from PIL import Image
from torchvision import transforms
from scipy.ndimage import zoom
from sklearn.decomposition import PCA
from sklearn.cluster import KMeans
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap


def resize_to_divisible(img, patch_size=16, max_size=768):
    """Resize image to be divisible by patch size."""
    w, h = img.size
    aspect_ratio = w / h
    
    if h > w:
        new_h = min(h, max_size)
        new_w = int(new_h * aspect_ratio)
    else:
        new_w = min(w, max_size)
        new_h = int(new_w / aspect_ratio)
    
    new_h = (new_h // patch_size) * patch_size
    new_w = (new_w // patch_size) * patch_size
    
    return transforms.Resize((new_h, new_w))(img)


def extract_cluster_statistics(cluster_labels_spatial, features_raw, features_pca_spatial, patch_h, patch_w, h_pixels, w_pixels, method_name):
    """Extract statistics for each cluster in the requested nested structure.
    
    Args:
        cluster_labels_spatial: Cluster labels (patch_h, patch_w)
        features_raw: Raw DINOv3 features (N_patches, 768)
        features_pca_spatial: PCA features (patch_h, patch_w, 3)
        patch_h: Height in patches
        patch_w: Width in patches
        h_pixels: Height in pixels
        w_pixels: Width in pixels
        
    Returns:
        Dictionary with nested structure containing metadata and clusters
    """
    unique_labels = np.unique(cluster_labels_spatial)
    unique_labels = unique_labels[unique_labels >= 0]
    
    clusters_list = []
    
    # Calculate scaling factors
    scale_y = h_pixels / patch_h
    scale_x = w_pixels / patch_w
    
    for label in unique_labels:
        mask = cluster_labels_spatial == label
        coords_patch = np.argwhere(mask)  # Returns (row, col) = (y, x) in PATCH space
        n_pixels = len(coords_patch)
        
        # Convert patch coordinates to pixel coordinates
        pixel_coords = coords_patch.copy().astype(float)
        pixel_coords[:, 0] = coords_patch[:, 0] * scale_y  # y
        pixel_coords[:, 1] = coords_patch[:, 1] * scale_x  # x
        pixel_coords = pixel_coords.astype(int)
        
        # Clip to bounds
        pixel_coords[:, 0] = np.clip(pixel_coords[:, 0], 0, h_pixels - 1)
        pixel_coords[:, 1] = np.clip(pixel_coords[:, 1], 0, w_pixels - 1)
        
        # Get features for this cluster (use patch-space mask)
        mask_flat = mask.flatten()
        cluster_features_raw = features_raw[mask_flat]
        
        # Compute only the median feature
        features_median = np.median(cluster_features_raw, axis=0)
        
        # Spatial statistics (now in pixel space)
        y_coords, x_coords = pixel_coords[:, 0], pixel_coords[:, 1]
        centroid = (float(np.mean(x_coords)), float(np.mean(y_coords)))
        
        # Build lean structure with only essential data
        cluster_data = {
            'n_pixels': n_pixels,
            'pixel_coords': pixel_coords,
            'features_raw': {
                'median': features_median,
            },
            'spatial': {
                'centroid': centroid,
            }
        }
        clusters_list.append(cluster_data)
    
    # Sort by number of pixels (descending)
    clusters_list.sort(key=lambda x: x['n_pixels'], reverse=True)
    
    # Build final structure
    cluster_data = {
        'clusters': clusters_list,
        'metadata': {
            'patch_shape': (patch_h, patch_w),
            'image_shape': (h_pixels, w_pixels),
            'total_clusters': len(clusters_list),
            'total_patches': patch_h * patch_w,
        }
    }
    
    return cluster_data


def save_cluster_stats_json(output_path, data_to_save):
    """Saves cluster statistics to a JSON file with only essential data."""
    import json
    
    lean_data = {
        'metadata': data_to_save['metadata'],
        'clusters': []
    }
    
    for cluster in data_to_save['clusters']:
        lean_cluster = {
            'features_raw': {
                'median': cluster['features_raw']['median'].tolist()
            },
            'n_pixels': cluster['n_pixels'],
            'pixel_coords': cluster['pixel_coords'].tolist(),
            'spatial': {
                'centroid': cluster['spatial']['centroid']
            }
        }
        lean_data['clusters'].append(lean_cluster)
    
    with open(output_path, 'w') as f:
        json.dump(lean_data, f, indent=2)


def extract_dinov3_features(image_path, checkpoint_path, dinov3_location="/home/kneepolean/sreeharsha/dinov3", 
                           output_dir=None, device=None, save_cluster_stats=True):
    """Extract DINOv3 features from bone image using k-means clustering.
    
    Args:
        image_path: Path to input image
        checkpoint_path: Path to DINOv3 checkpoint
        dinov3_location: Path to DINOv3 repository
        output_dir: Output directory (default: creates subdir in image location)
        device: Device to use (default: auto-detect cuda/cpu)
        save_cluster_stats: Whether to save cluster statistics as .json (default: True)
        
    Returns:
        dict with keys:
            - features: Raw features array (N_patches, 768)
            - pca_features: PCA features (patch_h, patch_w, 3)
            - image_size: (width, height) of processed image
            - patch_size: (patch_w, patch_h)
    """
    if device is None:
        device = "cuda" if torch.cuda.is_available() else "cpu"
    
    # Load model
    model = torch.hub.load(
        repo_or_dir=dinov3_location,
        model="dinov3_vitb16",
        source="local",
        pretrained=True,
        weights=checkpoint_path,
    )
    model.eval()
    model = model.to(device)
    
    # Load and preprocess image
    image = Image.open(image_path).convert("RGB")
    
    # Center crop to 848x480
    crop_width, crop_height = 848, 480
    img_width, img_height = image.size
    left = (img_width - crop_width) // 2
    top = (img_height - crop_height) // 2
    right = left + crop_width
    bottom = top + crop_height
    image = image.crop((left, top, right, bottom))
    
    transform = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize(mean=(0.485, 0.456, 0.406), std=(0.229, 0.224, 0.225))
    ])
    image_tensor = transform(image).unsqueeze(0).to(device)
    
    # Extract features
    start_time = time.time()
    with torch.inference_mode():
        features_dict = model.forward_features(image_tensor)
        features = features_dict['x_norm_patchtokens']
    
    if device == "cuda":
        torch.cuda.synchronize()
    
    forward_time = time.time() - start_time
    print(f"Forward pass: {forward_time*1000:.2f} ms")
    
    features_cpu = features.squeeze(0).cpu()
    
    # Calculate dimensions
    # PIL .size returns (width, height), we need (height, width) for numpy
    w_pixels, h_pixels = image.size
    patch_h = h_pixels // 16
    patch_w = w_pixels // 16
    
    # Compute 3D PCA features for visualization
    pca_3d = PCA(n_components=3, whiten=True)
    features_pca_3d = pca_3d.fit_transform(features_cpu.numpy())
    features_pca_spatial = features_pca_3d.reshape(patch_h, patch_w, 3)
    
    # Normalize PCA features for visualization
    for i in range(3):
        channel = features_pca_spatial[:, :, i]
        features_pca_spatial[:, :, i] = (channel - channel.min()) / (channel.max() - channel.min())
    
    # Compute 8D PCA features for clustering
    pca_8d = PCA(n_components=8, whiten=True)
    features_pca_8d = pca_8d.fit_transform(features_cpu.numpy())
    
    # Run k-means clustering on 8D PCA features
    cluster_start = time.time()
    clusterer_kmeans = KMeans(n_clusters=5, random_state=42, n_init=10)
    labels_kmeans = clusterer_kmeans.fit_predict(features_pca_8d)
    cluster_time = time.time() - cluster_start
    print(f"K-MEANS: {cluster_time*1000:.2f} ms (5 clusters)")
    
    cluster_labels_spatial = labels_kmeans.reshape(patch_h, patch_w)
    
    # Save outputs if output_dir specified
    if output_dir is not None:
        # model_name = os.path.splitext(os.path.basename(checkpoint_path))[0].split('_pretrain')[0]
        output_name = os.path.splitext(os.path.basename(image_path))[0]
        # output_path = os.path.join(output_dir, model_name, output_name)
        output_path = os.path.join(output_dir, output_name)
        
        # Upsample from patch space to pixel space for visualizations
        zoom_h = h_pixels / patch_h
        zoom_w = w_pixels / patch_w
        features_pca_upsampled = zoom(features_pca_spatial, (zoom_h, zoom_w, 1), order=3)
        image_array = np.array(image)
        cluster_labels_upsampled = zoom(cluster_labels_spatial.astype(float), (zoom_h, zoom_w), order=0)
        
        # Normalize PCA features for saving (0-255 uint8)
        features_pca_uint8 = (features_pca_upsampled * 255).astype(np.uint8)
        
        # Save PCA visualization using PIL for exact pixel dimensions
        Image.fromarray(features_pca_uint8).save(os.path.join(output_path, f"{output_name}_pca_vis.png"))
        
        # Create colormap for cluster labels
        cmap = plt.get_cmap('tab10')
        
        # Normalize cluster labels to 0-1 range for colormap
        def apply_colormap(labels):
            normalized = (labels - labels.min()) / (labels.max() - labels.min() + 1e-8)
            colored = cmap(normalized)[:, :, :3]  # Remove alpha channel
            return (colored * 255).astype(np.uint8)
        
        # Save k-means cluster visualizations
        cluster_kmeans_colored = apply_colormap(cluster_labels_upsampled)
        Image.fromarray(cluster_kmeans_colored).save(os.path.join(output_path, f"image_clusters_kmeans.png"))
        
        overlay_kmeans = (image_array * 0.5 + cluster_kmeans_colored * 0.5).astype(np.uint8)
        Image.fromarray(overlay_kmeans).save(os.path.join(output_path, f"image_clusters_overlay_kmeans.png"))
        
        # Extract cluster statistics
        cluster_data = extract_cluster_statistics(
            cluster_labels_spatial, 
            features_cpu.numpy(), 
            features_pca_spatial, 
            patch_h, 
            patch_w,
            h_pixels,
            w_pixels,
            'kmeans'
        )
        
        # Save cluster statistics as JSON
        if save_cluster_stats:
            json_path = os.path.join(output_path, f"image_features.json")
            save_cluster_stats_json(json_path, cluster_data)
    
    return {
        'features': features_cpu.numpy(),
        'pca_features': features_pca_spatial,
        'image_size': (w_pixels, h_pixels),
        'patch_size': (patch_w, patch_h)
    }


def main():
    parser = argparse.ArgumentParser(description="Extract DINOv3 features from bone images using k-means clustering")
    parser.add_argument("--dinov3-location", type=str, 
                       default="/home/kneepolean/sreeharsha/dinov3")
    parser.add_argument("--checkpoint-path", type=str, default="/checkpoints/dinov3_vitb16_pretrain_lvd1689m-73cec8be.pth")
    parser.add_argument("--image-path", type=str, required=True)
    parser.add_argument("--output-dir", type=str, default="/home/kneepolean/sreeharsha/bone_data/extracted_features")
    parser.add_argument("--save-cluster-stats", action="store_true",
                       help="Save cluster statistics as .json files (default: False)")
    args = parser.parse_args()
    
    result = extract_dinov3_features(
        args.image_path,
        args.checkpoint_path,
        args.dinov3_location,
        args.output_dir,
        save_cluster_stats=args.save_cluster_stats
    )
    
    print(f"Features extracted: {result['features'].shape}")
    print(f"PCA features: {result['pca_features'].shape}")
    print(f"Output saved to: {args.output_dir}")


if __name__ == "__main__":
    main()
