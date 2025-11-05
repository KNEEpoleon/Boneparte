
import os
import time
import pickle

import numpy as np
import torch
from PIL import Image
from torchvision import transforms
from scipy.ndimage import zoom
from sklearn.decomposition import PCA
from sklearn.cluster import MeanShift, estimate_bandwidth
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt



class DINOBoneExtractor():
    """
    Class to extract the bone using DINO.
    """
    def __init__(self, checkpoint_path, dinov3_location, output_home, save_cluster_stats = True):
        # self.image_path = image_path # this will change per call
        self.checkpoint_path = checkpoint_path # shouldn't change
        self.dinov3_location = dinov3_location # shouldn't change
        self.output_home = output_home # might not change
        self.output_dir = None
        self.save_visualizations = True # user configurable
        self.save_cluster_stats = save_cluster_stats # user configurable
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = self.load_dino_model()
    
    def make_output_dir(self, image_path):
        self.output_dir = os.path.join(self.output_home, os.path.splitext(os.path.basename(image_path))[0])
        if self.save_cluster_stats or self.save_visualizations:
            os.makedirs(self.output_dir, exist_ok=True)

    def get_centroid(self, image_path):
        # simply runs a DINO v3 inference.
        self.make_output_dir(image_path)

        dino_pca_features, dino_features, image_resized, h_patch, w_patch, h_pixels, w_pixels = self.get_dino_features(image_path)

           
        cluster_results = self._get_dino_clusters(dino_pca_features, h_patch, w_patch)

        
        if self.save_visualizations:    
            self._save_dino_features(dino_pca_features, "DINO_PCA_Features")
            self._save_dino_clusters(cluster_results['meanshift'], image_resized, "DINO_Clusters", h_pixels, w_pixels, h_patch, w_patch)
        
        if self.save_cluster_stats:
            # Extract cluster statistics
            cluster_data = self._extract_cluster_statistics(
                cluster_results['meanshift'], 
                dino_features, 
                dino_pca_features, 
                h_patch, w_patch, h_pixels, w_pixels, 'meanshift'
            )
            
            # Save cluster statistics as pickle
            with open(os.path.join(self.output_dir, "cluster_stats_meanshift.pkl"), 'wb') as f:
                pickle.dump(cluster_data, f)

        # Return placeholder - implement centroid computation later
        return {
            'clusters': cluster_results,
            'features_shape': (h_patch, w_patch),
            'image_shape': (h_pixels, w_pixels),
            'centroid': None  # TODO: implement centroid computation
        }


    def load_dino_model(self):
        """Load the DINOv3 model."""
        model = torch.hub.load(
            repo_or_dir=self.dinov3_location,
            model="dinov3_vitb16",
            source="local",
            pretrained=True,
            weights=self.checkpoint_path,
        )
        model.eval()
        model = model.to(self.device)
        return model

    def get_dino_features(self, image_path):
        """Extract DINOv3 features from bone image.
        
        Args:
            image_path: Path to input image
            save_cluster_stats: Whether to save cluster statistics as .pkl (default: True)
            
        Returns:
            dict with keys:
                - features: Raw features array (N_patches, 768)
                - pca_features: PCA features (patch_h, patch_w, 3)
                - image_size: (width, height) of processed image
                - patch_size: (patch_w, patch_h)
                - cluster_results: dict of cluster labels by method
        """
        # Load and preprocess image
        image = Image.open(image_path).convert("RGB")
        image_resized = self._resize_to_divisible(image, patch_size=16, max_size=768)
        
        transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(mean=(0.485, 0.456, 0.406), std=(0.229, 0.224, 0.225))
        ])
        image_tensor = transform(image_resized).unsqueeze(0).to(self.device)
        
        # Extract features
        start_time = time.time()
        with torch.inference_mode():
            features_dict = self.model.forward_features(image_tensor)
            features = features_dict['x_norm_patchtokens']
        
        if self.device == "cuda":
            torch.cuda.synchronize()
        
        forward_time = time.time() - start_time
        print(f"Forward pass: {forward_time*1000:.2f} ms")
        
        features_cpu = features.squeeze(0).cpu()
        
        # Calculate dimensions
        # PIL .size returns (width, height), we need (height, width) for numpy
        w_pixels, h_pixels = image_resized.size
        h_patch = h_pixels // 16
        w_patch = w_pixels // 16
        
        # Compute PCA features
        pca = PCA(n_components=3, whiten=True)
        features_pca = pca.fit_transform(features_cpu.numpy())
        features_pca_spatial = features_pca.reshape(h_patch, w_patch, 3)
        
        # Normalize PCA features
        for i in range(3):
            channel = features_pca_spatial[:, :, i]
            features_pca_spatial[:, :, i] = (channel - channel.min()) / (channel.max() - channel.min())
 
        return features_pca_spatial, features_cpu.numpy(), image_resized, h_patch, w_patch, h_pixels, w_pixels

    def _save_dino_features(self, features_pca_spatial, output_name):
        """Save PCA features at native DINO patch resolution."""
        plt.figure(figsize=(10, 8))
        plt.imshow(features_pca_spatial)
        plt.axis('off')
        plt.savefig(os.path.join(self.output_dir, f"{output_name}_pca_vis.png"), dpi=300, bbox_inches='tight')
        plt.close()

    def _get_dino_clusters(self, features_pca, h_patch, w_patch):
        # Running Meanshift
        cluster_results = {}
        cluster_start = time.time()
        
        # Reshape from (h_patch, w_patch, 3) to (n_patches, 3) for sklearn
        features_pca_flat = features_pca.reshape(-1, 3)
        
        bandwidth = estimate_bandwidth(features_pca_flat, quantile=0.2, n_samples=200)
        clusterer = MeanShift(bandwidth=bandwidth, bin_seeding=True)
        labels = clusterer.fit_predict(features_pca_flat)
        
        cluster_time = time.time() - cluster_start
        n_clusters_found = len(np.unique(labels[labels >= 0]))
        print(f"Meanshift: {cluster_time*1000:.2f} ms ({n_clusters_found} clusters)")
        
        # Reshape labels back to spatial dimensions
        cluster_results['meanshift'] = labels.reshape(h_patch, w_patch)

        return cluster_results

    def _save_dino_clusters(self, cluster_labels_spatial, image_resized, output_name, h_pixels, w_pixels, h_patch, w_patch):
        """Save clusters overlayed on full resolution image (with upsampling)."""
        from scipy.ndimage import zoom
        
        # Upsample clusters to match image resolution
        zoom_h = h_pixels / h_patch
        zoom_w = w_pixels / w_patch
        cluster_labels_upsampled = zoom(cluster_labels_spatial.astype(float), (zoom_h, zoom_w), order=0)
        
        # Convert PIL image to numpy array
        image_resized_array = np.array(image_resized)
        
        # Save cluster overlay on RGB image (like your working code)
        plt.figure(figsize=(10, 8))
        plt.imshow(image_resized_array)
        plt.imshow(cluster_labels_upsampled, cmap='tab10', alpha=0.5, interpolation='nearest')
        plt.axis('off')
        plt.savefig(os.path.join(self.output_dir, f"{output_name}.png"), dpi=300, bbox_inches='tight')
        plt.close()
    
    def compute_similarity():
        pass

    def _resize_to_divisible(self, img, patch_size=16, max_size=768):
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
 

    def _extract_cluster_statistics(self, cluster_labels_spatial, features_raw, features_pca_spatial, h_patch, w_patch, h_pixels, w_pixels, method_name):
        """Extract statistics for each cluster in the requested nested structure.
        
        Args:
            cluster_labels_spatial: Cluster labels (h_patch, w_patch)
            features_raw: Raw DINOv3 features (N_patches, 768)
            features_pca_spatial: PCA features (h_patch, w_patch, 3)
            h_patch: Height in patches
            w_patch: Width in patches
            h_pixels: Height in pixels
            w_pixels: Width in pixels
            method_name: Clustering method name
            
        Returns:
            Dictionary with nested structure containing metadata and clusters
        """
        unique_labels = np.unique(cluster_labels_spatial)
        unique_labels = unique_labels[unique_labels >= 0]  # Remove noise label if present
        
        clusters_list = []
        features_pca_flat = features_pca_spatial.reshape(-1, 3)
        
        # Calculate scaling factors
        scale_y = h_pixels / h_patch
        scale_x = w_pixels / w_patch
        
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
            cluster_features_pca = features_pca_flat[mask_flat]
            
            # Compute robust statistics on raw features
            features_median = np.median(cluster_features_raw, axis=0)
            features_p25 = np.percentile(cluster_features_raw, 25, axis=0)
            features_p75 = np.percentile(cluster_features_raw, 75, axis=0)
            
            # Trimmed mean (10-90 percentile)
            p10 = np.percentile(cluster_features_raw, 10, axis=0)
            p90 = np.percentile(cluster_features_raw, 90, axis=0)
            mask_trimmed = np.all((cluster_features_raw >= p10) & (cluster_features_raw <= p90), axis=1)
            features_trimmed_mean = np.mean(cluster_features_raw[mask_trimmed], axis=0) if np.any(mask_trimmed) else features_median
            
            # PCA statistics (for visualization)
            pca_median = np.median(cluster_features_pca, axis=0)
            pca_mean = np.mean(cluster_features_pca, axis=0)
            
            # Spatial statistics (now in pixel space)
            y_coords, x_coords = pixel_coords[:, 0], pixel_coords[:, 1]
            centroid = (float(np.mean(x_coords)), float(np.mean(y_coords)))
            bbox = (int(np.min(x_coords)), int(np.min(y_coords)), 
                    int(np.max(x_coords)), int(np.max(y_coords)))
            
            # Build nested structure
            cluster_data = {
                'id': int(label),
                'n_pixels': n_pixels,
                'pixel_coords': pixel_coords,  # (N, 2) array of (y, x)
                'features_raw': {
                    'median': features_median,
                    'p25': features_p25,
                    'p75': features_p75,
                    'mean_trimmed': features_trimmed_mean,
                },
                'features_pca': {
                    'median': pca_median,
                    'mean': pca_mean,
                },
                'spatial': {
                    'centroid': centroid,
                    'bbox': bbox,
                }
            }
            clusters_list.append(cluster_data)
        
        # Sort by number of pixels (descending)
        clusters_list.sort(key=lambda x: x['n_pixels'], reverse=True)
        
        # Build final structure
        cluster_data = {
            'method': method_name,
            'clusters': clusters_list,
            'metadata': {
                'patch_shape': (h_patch, w_patch),
                'image_shape': (h_pixels, w_pixels),
                'total_clusters': len(clusters_list),
                'total_patches': h_patch * w_patch,
            }
        }
        
        return cluster_data 
        

    @property
    def load_codebook():
        pass

    @staticmethod
    def load_cluster_features():
        pass

    @staticmethod
    def visualize_feature_clusters():
        pass

    @staticmethod
    def visualize_camera_flow():
        pass