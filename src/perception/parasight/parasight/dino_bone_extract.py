    
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
    def __init__(self, checkpoint_path, dinov3_path, codebook_path, output_home, save_cluster_stats = True):
        self.checkpoint_path = checkpoint_path # shouldn't change
        self.dinov3_path = dinov3_path # shouldn't change
        self.output_home = output_home # might not change
        self.output_dir = None
        self.save_visualizations = True # user configurable
        self.save_cluster_stats = False #user figurable
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = self.load_dino_model()
        self.codebook = self.load_bone_features(codebook_path)
        self.threshold = 0.7
        self.min_size = 100

    def load_bone_features(self, pkl_path):
        """Load bone features from pickle file."""
        with open(pkl_path, 'rb') as f:
            return pickle.load(f)
    
    def make_output_dir(self, image_path):
        """Make output folder for query image."""
        self.output_dir = os.path.join(self.output_home, os.path.splitext(os.path.basename(image_path))[0])
        if self.save_cluster_stats or self.save_visualizations:
            os.makedirs(self.output_dir, exist_ok=True)

    def load_dino_model(self):
        """Load the DINOv3 model."""
        model = torch.hub.load(
            repo_or_dir=self.dinov3_path,
            model="dinov3_vitb16",
            source="local",
            pretrained=True,
            weights=self.checkpoint_path,
        )
        model.eval()
        model = model.to(self.device)
        return model

    def _save_dino_features(self, features_pca_spatial, output_name):
        """Save PCA features at native DINO patch resolution."""
        plt.figure(figsize=(10, 8))
        plt.imshow(features_pca_spatial)
        plt.axis('off')
        plt.savefig(os.path.join(self.output_dir, f"{output_name}_pca_vis.png"), dpi=300, bbox_inches='tight')
        plt.close()


    def get_centroid(self, image_path):
        """
        Computes the location of the bone centroid in the image plane.

        Args:
            - image_path: Path to the query image: maybe change in the future to accept a numpy array.
        Returns:
            - bone centroid in the pixel space IF it exists
        """
        self.make_output_dir(image_path)
        
        # Track overall timing
        start_time = time.time()
        dino_pca_features, dino_features, image_resized, h_patch, w_patch, h_pixels, w_pixels = self.get_dino_features(image_path)
        cluster_data = self._get_dino_clusters(dino_pca_features, h_patch, w_patch)

        cluster_statistics = self._extract_cluster_statistics(
            cluster_data['meanshift'], 
            dino_features, 
            dino_pca_features, 
            h_patch, w_patch, h_pixels, w_pixels
        )

        detected_bone = self.detect_bone(cluster_statistics, image_resized, image_path)
        
        total_time = time.time() - start_time
        print(f"Total detection time: {total_time*1000:.1f} ms")

        if self.save_visualizations:    
            self._save_dino_features(dino_pca_features, "DINO_PCA_Features")
            self._save_dino_clusters(cluster_data['meanshift'], image_resized, "DINO_Clusters", h_pixels, w_pixels, h_patch, w_patch)
        
        if self.save_cluster_stats:
            with open(os.path.join(self.output_dir, "cluster_stats_meanshift.pkl"), 'wb') as f:
                pickle.dump(cluster_statistics, f)

        return detected_bone

    def get_dino_features(self, image_path):
        """Extract DINOv3 features from bone image.
        
        Args:
            image_path: Path to input image
            
        Returns:
            Tuple of:
                - features_pca_spatial: PCA features (h_patch, w_patch, 3)
                - features_raw: Raw DINO features (n_patches, 768)
                - image_resized: Processed PIL image
                - h_patch: Height in patches
                - w_patch: Width in patches
                - h_pixels: Height in pixels
                - w_pixels: Width in pixels
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
        print(f"DINO forward pass: {forward_time*1000:.1f} ms")
        
        features_cpu = features.squeeze(0).cpu()
        
        # Calculate dimensions
        # PIL .size returns (width, height), we need (height, width) for numpy
        w_pixels, h_pixels = image_resized.size
        h_patch = h_pixels // 16
        w_patch = w_pixels // 16
        
        # Compute PCA features
        pca = PCA(n_components=3, whiten=True)
        features_pca_spatial = pca.fit_transform(features_cpu.numpy()).reshape(h_patch, w_patch, 3)
        
        # Normalize PCA features
        for i in range(3):
            channel = features_pca_spatial[:, :, i]
            features_pca_spatial[:, :, i] = (channel - channel.min()) / (channel.max() - channel.min())
 
        return features_pca_spatial, features_cpu.numpy(), image_resized, h_patch, w_patch, h_pixels, w_pixels

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
        print(f"MeanShift clustering: {cluster_time*1000:.1f} ms ({n_clusters_found} clusters)")
        
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
 

    def _extract_cluster_statistics(self, cluster_labels_spatial, features_raw, features_pca_spatial, h_patch, w_patch, h_pixels, w_pixels):
        """Extract statistics for each cluster in the requested nested structure.
        
        Args:
            cluster_labels_spatial: Cluster labels (h_patch, w_patch)
            features_raw: Raw DINOv3 features (N_patches, 768)
            features_pca_spatial: PCA features (h_patch, w_patch, 3)
            h_patch: Height in patches
            w_patch: Width in patches
            h_pixels: Height in pixels
            w_pixels: Width in pixels
            
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
            'method': 'meanshift',
            'clusters': clusters_list,
            'metadata': {
                'patch_shape': (h_patch, w_patch),
                'image_shape': (h_pixels, w_pixels),
                'total_clusters': len(clusters_list),
                'total_patches': h_patch * w_patch,
            }
        }
        
        return cluster_data 
            

    def compute_similarity(self, query_features, ref_features):
        """Compute cosine similarity between query and reference features.
        
        Args:
            query_features: Query cluster's features_raw dict
            ref_features: Reference features dict (from codebook or bone_features)
            
        Returns:
            Similarity score (0-1)
        """
        q = query_features['median']
        r = ref_features['median']
        return np.dot(q, r) / (np.linalg.norm(q) * np.linalg.norm(r))
        

    def detect_bone(self, cluster_statistics, image_resized, image_path):
        """Detect bone cluster in a new image using codebook.
        
        Args:
            image_path: Path to RGB image
            min_size: Minimum cluster size to consider
            threshold: Minimum similarity threshold
        """

        # Extract codebook embeddings
        if 'embeddings' in self.codebook:
            ref_features = self.codebook['embeddings']
        else:
            ref_features = self.codebook['bone_cluster']['features_raw']
        
        # Compute similarities for all clusters
        scores = []
        for i, cluster in enumerate(cluster_statistics['clusters']):
            if cluster['n_pixels'] < self.min_size:
                scores.append((i, -1.0, 'too_small'))
                continue
            sim = self.compute_similarity(cluster['features_raw'], ref_features)
            reason = 'valid' if sim >= self.threshold else 'below_threshold'
            scores.append((i, sim, reason))
        
        scores.sort(key=lambda x: x[1], reverse=True)
        best_idx, best_sim, best_reason = scores[0]
        best_cluster = cluster_statistics['clusters'][best_idx]
        
        # Check if detection is reliable
        if best_sim < self.threshold:
            print(f"✗ No bone detected (best similarity: {best_sim:.3f} < threshold: {self.threshold})")
            
            # Save rejection status
            os.makedirs(self.output_dir, exist_ok=True)
            output_name = os.path.splitext(os.path.basename(image_path))[0]
            rejection_path = os.path.join(self.output_dir, f"{output_name}_NO_DETECTION.txt")
            with open(rejection_path, 'w') as f:
                f.write(f"NO BONE DETECTED\n")
                f.write(f"Image: {os.path.basename(image_path)}\n")
                f.write(f"Best similarity: {best_sim:.4f}\n")
                f.write(f"Threshold: {self.threshold:.4f}\n")
                f.write(f"Reason: All clusters below confidence threshold\n")
                f.write(f"\nTop 3 candidates:\n")
                for i, (idx, sim, reason) in enumerate(scores[:3]):
                    f.write(f"  {i+1}. Cluster {idx}: {sim:.4f}\n")
            return None
        
        # Valid detection
        print(f"✓ Bone detected (similarity: {best_sim:.3f})")
        
        # Compute 2D bone centroid (median of bone cluster points)
        coords = cluster_statistics['clusters'][best_idx]['pixel_coords']  # Shape: (N, 2) with [row, col] format
        
        # Compute median in DINO-resized pixel space
        bone_centroid_row_dino = np.median(coords[:, 0])
        bone_centroid_col_dino = np.median(coords[:, 1])
        
        # Load original image to get original dimensions
        image = Image.open(image_path).convert('RGB')
        w_original, h_original = image.size
        
        # Get DINO-resized dimensions
        h_dino, w_dino = cluster_statistics['metadata']['image_shape']
        
        # Scale from DINO space to original image space
        scale_u = w_original / w_dino
        scale_v = h_original / h_dino
        bone_centroid_col_original = bone_centroid_col_dino * scale_u
        bone_centroid_row_original = bone_centroid_row_dino * scale_v
        
        # Convert to original image space (for depth map alignment)
        bone_centroid_2d = (float(bone_centroid_col_original), float(bone_centroid_row_original))
        bone_centroid_2d_dino = (float(bone_centroid_col_dino), float(bone_centroid_row_dino))
        
        # Calculate distance from image center (in original image space)
        image_center_original = (w_original / 2.0, h_original / 2.0)
        distance_from_center = float(np.linalg.norm(np.array(bone_centroid_2d) - np.array(image_center_original)))
        
        print(f"  Bone centroid: ({bone_centroid_2d[0]:.1f}, {bone_centroid_2d[1]:.1f}) px in {w_original}×{h_original} image")
        print(f"  Distance from center: {distance_from_center:.1f} px")
        
        # Visualize detection (using DINO space for visualization)
        
        # Resize to match DINOv3 processed size
        h_target, w_target = cluster_statistics['metadata']['image_shape']
        image_resized = image.resize((w_target, h_target), Image.LANCZOS)
        img_array = np.array(image_resized)
        
        # Compute image center
        image_center_u = w_target / 2.0
        image_center_v = h_target / 2.0
        
        fig, axes = plt.subplots(1, 2, figsize=(16, 8))
        
        # Left: Top 3 candidates
        ax = axes[0]
        ax.imshow(img_array)
        colors = ['red', 'green', 'blue']
        
        for rank, (cluster_idx, sim, reason) in enumerate(scores[:3]):
            if sim < 0:
                continue
            cluster = cluster_statistics['clusters'][cluster_idx]
            coords = cluster['pixel_coords']
            
            # Draw circles at each coordinate (s=100 ≈ 5-6 pixel radius)
            ax.scatter(coords[:, 1], coords[:, 0], c=colors[rank], s=100, alpha=0.6, edgecolors='none')
            
            # Label with similarity score
            centroid = cluster_statistics['clusters'][cluster_idx]['spatial']['centroid']
            ax.text(centroid[0], centroid[1], f"{rank+1}\n{sim:.3f}",
                color='white', fontsize=14, weight='bold',
                ha='center', va='center',
                bbox=dict(boxstyle='round,pad=0.5', facecolor='black', alpha=0.8))
        
        ax.set_title('Top 3 Candidates', fontsize=14)
        ax.axis('off')
        
        # Right: Best detection with arrow
        ax = axes[1]
        ax.imshow(img_array)
        coords = cluster_statistics['clusters'][best_idx]['pixel_coords']
        
        # Draw red circles for detected bone (s=100 ≈ 5-6 pixel radius)
        ax.scatter(coords[:, 1], coords[:, 0], c='red', s=100, alpha=0.7, edgecolors='none')
        
        # Draw arrow from image center to bone centroid
        # Using a subtle, low-gaudy style
        arrow_props = dict(
            arrowstyle='-|>',
            lw=2,
            color='cyan',
            alpha=0.7,
            shrinkA=0,
            shrinkB=0
        )
        
        # Draw the arrow (using DINO space coordinates for visualization)
        ax.annotate('', 
                    xy=(bone_centroid_2d_dino[0], bone_centroid_2d_dino[1]),  # End point (bone centroid)
                    xytext=(image_center_u, image_center_v),  # Start point (image center)
                    arrowprops=arrow_props)
        
        # Mark the bone centroid with a small marker
        ax.plot(bone_centroid_2d_dino[0], bone_centroid_2d_dino[1], 
                'c*', markersize=15, markeredgecolor='white', markeredgewidth=1.5)
        
        status_text = "DETECTED" if best_sim >= self.threshold else "LOW CONFIDENCE"
        ax.set_title(f'{status_text}: Similarity {best_sim:.3f}', fontsize=14)
        ax.axis('off')
        
        plt.tight_layout()
        
        # Save results
        os.makedirs(self.output_dir, exist_ok=True)
        output_name = os.path.splitext(os.path.basename(image_path))[0]
        
        if self.save_visualizations:
            viz_path = os.path.join(self.output_dir, f"{output_name}_detection.png")
            plt.savefig(viz_path, dpi=150, bbox_inches='tight')
        
        # Save detected bone
        detected_bone = {
            'centroid': {'u': bone_centroid_2d[0], 'v': bone_centroid_2d[1]},
            'distance_from_center_px': distance_from_center,
            'similarity': float(best_sim),
            'source_image': os.path.basename(image_path),
            'codebook_version': self.codebook.get('version', 'unknown')
        }
        
        detected_path = os.path.join(self.output_dir, f"{output_name}_detected_bone.pkl")
        with open(detected_path, 'wb') as f:
            pickle.dump(detected_bone, f)
        
        # plt.show()  # Commented out - causes warning in non-interactive environments
        return detected_bone

    @staticmethod
    def visualize_feature_clusters():
        pass

    @staticmethod
    def visualize_camera_flow():
        pass