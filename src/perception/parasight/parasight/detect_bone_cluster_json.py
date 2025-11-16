#!/usr/bin/env python3
"""Build bone codebook and detect bone clusters in new images."""

import argparse
import os
import glob
import json

import numpy as np
import matplotlib.pyplot as plt
from PIL import Image


def load_cluster_data(json_path):
    """Loads cluster data from a JSON file."""
    with open(json_path, 'r') as f:
        data = json.load(f)
    
    for cluster in data['clusters']:
        cluster['features_raw']['median'] = np.array(cluster['features_raw']['median'])
        cluster['pixel_coords'] = np.array(cluster['pixel_coords'])
    
    return data


def compute_similarity(query_median, ref_median):
    """Compute cosine similarity between query and reference median features.
    
    Args:
        query_median: Query cluster's median feature vector
        ref_median: Reference median feature vector (from codebook)
        
    Returns:
        Similarity score (0-1)
    """
    return np.dot(query_median, ref_median) / (np.linalg.norm(query_median) * np.linalg.norm(ref_median))


def build_codebook(bone_features_dir, output_path):
    """Build bone codebook from multiple annotated bone features.
    
    Args:
        bone_features_dir: Directory containing bone_features.json files
        output_path: Output path for bone_codebook.json
    """
    base_path, _ = os.path.splitext(output_path)
    output_path = base_path + '.json'

    pattern = os.path.join(bone_features_dir, "**", "bone_features.json")
    bone_files = glob.glob(pattern, recursive=True)
    
    if len(bone_files) == 0:
        print(f"Error: No bone_features.json files found in {bone_features_dir}")
        return
    
    print(f"Found {len(bone_files)} annotated bone samples")
    
    bones = []
    for bf in bone_files:
        try:
            bone_cluster_data = load_cluster_data(bf)
            if bone_cluster_data['clusters']:
                largest_cluster = bone_cluster_data['clusters'][0]
                bones.append(largest_cluster)
                print(f"  ✓ {os.path.dirname(bf)} (using largest cluster)")
            else:
                print(f"  ✗ No clusters found in {bf}")
        except Exception as e:
            print(f"  ✗ Failed to load or process {bf}: {e}")
    
    if len(bones) == 0:
        print("Error: No valid bone features loaded")
        return
    
    print(f"\nAnalyzing {len(bones)} bone samples...")
    
    medians = np.array([b['features_raw']['median'] for b in bones])
    
    n = len(bones)
    similarities = []
    for i in range(n):
        for j in range(i+1, n):
            sim = compute_similarity(medians[i], medians[j])
            similarities.append(sim)
    
    similarities = np.array(similarities)
    
    codebook_median = np.mean(medians, axis=0)
    
    codebook = {
        'version': '1.0',
        'n_training_samples': len(bones),
        'method': 'averaged_median',
        'embedding': codebook_median.tolist(),
        'statistics': {
            'mean_intra_similarity': float(np.mean(similarities)),
            'std_similarity': float(np.std(similarities)),
            'min_similarity': float(np.min(similarities)),
            'max_similarity': float(np.max(similarities)),
        },
        'source_files': [os.path.basename(f) for f in bone_files],
    }
    
    with open(output_path, 'w') as f:
        json.dump(codebook, f, indent=2)
    
    print("\n" + "="*60)
    print("BONE CODEBOOK STATISTICS")
    print("="*60)
    print(f"Training samples: {len(bones)}")
    print(f"Mean pairwise similarity: {codebook['statistics']['mean_intra_similarity']:.4f}")
    print(f"Std deviation: {codebook['statistics']['std_similarity']:.4f}")
    print(f"Min similarity: {codebook['statistics']['min_similarity']:.4f}")
    print(f"Max similarity: {codebook['statistics']['max_similarity']:.4f}")
    print(f"\n✓ Codebook saved to: {output_path}")
    print("="*60)
    
    plt.figure(figsize=(10, 6))
    plt.hist(similarities, bins=20, edgecolor='black', alpha=0.7)
    plt.axvline(np.mean(similarities), color='red', linestyle='--', 
                label=f'Mean: {np.mean(similarities):.3f}')
    plt.xlabel('Pairwise Similarity', fontsize=12)
    plt.ylabel('Count', fontsize=12)
    plt.title('Bone Feature Similarity Distribution', fontsize=14)
    plt.legend()
    plt.grid(alpha=0.3)
    plt.tight_layout()
    
    plot_path = base_path + '_similarity_dist.png'
    plt.savefig(plot_path, dpi=150)
    print(f"✓ Similarity plot saved to: {plot_path}")
    plt.show()


def detect_bone(codebook_path, cluster_stats_path, image_path, output_dir, 
                min_size=100, threshold=0.75):
    """Detect bone cluster in a new image using codebook.
    
    Args:
        codebook_path: Path to bone_codebook.json
        cluster_stats_path: Path to cluster_stats_meanshift.json
        image_path: Path to RGB image
        output_dir: Directory to save detection results
        min_size: Minimum cluster size to consider
        threshold: Minimum similarity threshold
    """
    print("Loading codebook...")
    with open(codebook_path, 'r') as f:
        codebook = json.load(f)
    ref_median = np.array(codebook['embedding'])
    
    print("Loading query clusters...")
    cluster_data = load_cluster_data(cluster_stats_path)
    
    print(f"\nAnalyzing {len(cluster_data['clusters'])} clusters...")
    
    scores = []
    for i, cluster in enumerate(cluster_data['clusters']):
        if cluster['n_pixels'] < min_size:
            scores.append((i, -1.0, 'too_small'))
            continue
        
        query_median = cluster['features_raw']['median']
        sim = compute_similarity(query_median, ref_median)
        reason = 'valid' if sim >= threshold else 'below_threshold'
        scores.append((i, sim, reason))
    
    # Sort by similarity
    scores.sort(key=lambda x: x[1], reverse=True)
    
    # Print results
    print("\n" + "="*60)
    print("DETECTION RESULTS (sorted by similarity)")
    print("="*60)
    for i, (cluster_idx, sim, reason) in enumerate(scores[:5]):
        cluster = cluster_data['clusters'][cluster_idx]
        status = "✓" if reason == 'valid' else "✗"
        print(f"{status} Rank {i+1}: Cluster {cluster_idx}")
        print(f"   Similarity: {sim:.4f}")
        print(f"   Size: {cluster['n_pixels']} pixels")
        print(f"   Reason: {reason}")
    
    # Get best match
    best_idx, best_sim, best_reason = scores[0]
    best_cluster = cluster_data['clusters'][best_idx]
    
    print("\n" + "="*60)
    print("BONE DETECTION RESULTS")
    print("="*60)
    
    # Check if detection is reliable
    if best_sim < threshold:
        print(f"✗ NO BONE DETECTED")
        print(f"  Best candidate similarity: {best_sim:.4f}")
        print(f"  Required threshold: {threshold:.4f}")
        print(f"  Status: All clusters below confidence threshold")
        print("="*60)
        
        # Save rejection status
        output_name = os.path.splitext(os.path.basename(image_path))[0]
        rejection_path = os.path.join(output_dir, f"{output_name}_NO_DETECTION.txt")
        with open(rejection_path, 'w') as f:
            f.write(f"NO BONE DETECTED\n")
            f.write(f"Image: {os.path.basename(image_path)}\n")
            f.write(f"Best similarity: {best_sim:.4f}\n")
            f.write(f"Threshold: {threshold:.4f}\n")
            f.write(f"Reason: All clusters below confidence threshold\n")
            f.write(f"\nTop 3 candidates:\n")
            for i, (idx, sim, reason) in enumerate(scores[:3]):
                f.write(f"  {i+1}. Cluster {idx}: {sim:.4f}\n")
        
        print(f"\n✗ Rejection report saved to: {rejection_path}")
        print("\n⚠️  WARNING: No reliable bone detection in this image")
        return  # Exit without visualization
    
    # Valid detection
    print(f"✓ BONE DETECTED: Cluster {best_idx}")
    print(f"  Confidence: {best_sim:.4f}")
    print(f"  Size: {best_cluster['n_pixels']} pixels")
    print("="*60)
    
    # Compute 2D bone centroid (median of bone cluster points)
    coords = best_cluster['pixel_coords']  # Shape: (N, 2) with [row, col] format
    
    # Compute median in pixel space
    bone_centroid_row = np.median(coords[:, 0])
    bone_centroid_col = np.median(coords[:, 1])
    
    # RealSense pixel coordinate convention: (u, v) = (col, row)
    # u = horizontal axis (width), v = vertical axis (height)
    bone_centroid_2d = (float(bone_centroid_col), float(bone_centroid_row))
    
    print(f"\n2D Bone Centroid (RealSense convention):")
    print(f"  u (horizontal): {bone_centroid_2d[0]:.2f} pixels")
    print(f"  v (vertical):   {bone_centroid_2d[1]:.2f} pixels")
    
    # Visualize detection
    image = Image.open(image_path).convert('RGB')
    
    # Resize to match DINOv3 processed size
    h_target, w_target = cluster_data['metadata']['image_shape']
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
        cluster = cluster_data['clusters'][cluster_idx]
        coords = cluster['pixel_coords']
        
        # Draw circles at each coordinate (s=100 ≈ 5-6 pixel radius)
        ax.scatter(coords[:, 1], coords[:, 0], c=colors[rank], s=100, alpha=0.6, edgecolors='none')
        
        # Label with similarity score
        centroid = cluster['spatial']['centroid']
        ax.text(centroid[0], centroid[1], f"{rank+1}\n{sim:.3f}",
               color='white', fontsize=14, weight='bold',
               ha='center', va='center',
               bbox=dict(boxstyle='round,pad=0.5', facecolor='black', alpha=0.8))
    
    ax.set_title('Top 3 Candidates', fontsize=14)
    ax.axis('off')
    
    # Right: Best detection with arrow
    ax = axes[1]
    ax.imshow(img_array)
    coords = best_cluster['pixel_coords']
    
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
    
    # Draw the arrow
    ax.annotate('', 
                xy=(bone_centroid_2d[0], bone_centroid_2d[1]),  # End point (bone centroid)
                xytext=(image_center_u, image_center_v),  # Start point (image center)
                arrowprops=arrow_props)
    
    # Mark the bone centroid with a small marker
    ax.plot(bone_centroid_2d[0], bone_centroid_2d[1], 
            'c*', markersize=15, markeredgecolor='white', markeredgewidth=1.5)
    
    status_text = "DETECTED" if best_sim >= threshold else "LOW CONFIDENCE"
    ax.set_title(f'{status_text}: Similarity {best_sim:.3f}', fontsize=14)
    ax.axis('off')
    
    plt.tight_layout()
    
    # Save results
    output_name = os.path.splitext(os.path.basename(image_path))[0]
    viz_path = os.path.join(output_dir, f"{output_name}_detection.png")
    plt.savefig(viz_path, dpi=150, bbox_inches='tight')
    print(f"\n✓ Visualization saved to: {viz_path}")
    
    # Save detected bone
    detected_bone = {
        'detected_cluster': best_cluster,
        'confidence': float(best_sim),
        'source_image': os.path.basename(image_path),
        'codebook_version': codebook.get('version', 'unknown'),
        'all_scores': [(int(idx), float(sim)) for idx, sim, _ in scores],
        'distance_from_image_center': float(np.linalg.norm(np.array([image_center_u, image_center_v]) - np.array(bone_centroid_2d))),
        '2d_bone_centroid': {
            'u': bone_centroid_2d[0],  # Horizontal (column) in pixels
            'v': bone_centroid_2d[1],  # Vertical (row) in pixels
            'convention': 'RealSense (u=horizontal, v=vertical)',
            'image_size': {'width': w_target, 'height': h_target}
        }
    }
    
    print(f" The 2D bone centroid is {bone_centroid_2d}, image plane centroid is {image_center_u}, {image_center_v}, length of displacement is: {np.linalg.norm(np.array([image_center_u, image_center_v]) - np.array(bone_centroid_2d))}")
    detected_bone['detected_cluster']['features_raw']['median'] = detected_bone['detected_cluster']['features_raw']['median'].tolist()
    detected_bone['detected_cluster']['pixel_coords'] = detected_bone['detected_cluster']['pixel_coords'].tolist()
    
    detected_path = os.path.join(output_dir, f"{output_name}_detected_bone.json")
    with open(detected_path, 'w') as f:
        json.dump(detected_bone, f, indent=2)
    print(f"✓ Detection data saved to: {detected_path}")
    
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Build bone codebook or detect bones")
    subparsers = parser.add_subparsers(dest='mode', help='Operation mode')
    
    # Build codebook mode
    build_parser = subparsers.add_parser('build', help='Build bone codebook from annotations')
    build_parser.add_argument("--bone-dir", type=str, required=True,
                             help="Directory containing bone_features.json files")
    build_parser.add_argument("--output", type=str, required=True,
                             help="Output path for bone_codebook.json")
    
    # Detect mode
    detect_parser = subparsers.add_parser('detect', help='Detect bone in new image')
    detect_parser.add_argument("--codebook", type=str, 
                              default="/home/kneepolean/sreeharsha/bone_data/extracted_features_dataset/fvd_bone_codebook.json",
                              help="Path to bone_codebook.json (default: /home/kneepolean/sreeharsha/bone_data/extracted_features_dataset/fvd_bone_codebook.json)")
    
    # Create mutually exclusive group for input method
    input_group = detect_parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument("--image-name", type=str,
                            help="Image name only (e.g., 'bottom_0pin_3_Color'). Derives paths automatically.")
    input_group.add_argument("--cluster-stats", type=str,
                            help="Path to cluster_stats_meanshift.json (alternative to --image-name)")
    
    detect_parser.add_argument("--image", type=str,
                              help="Path to RGB image (required with --cluster-stats, auto-derived with --image-name)")
    detect_parser.add_argument("--output-dir", type=str, default="./detections",
                              help="Output directory for results")
    detect_parser.add_argument("--min-size", type=int, default=100,
                              help="Minimum cluster size (default: 100)")
    detect_parser.add_argument("--threshold", type=float, default=0.70,
                              help="Similarity threshold for detection (default: 0.70, recommended range: 0.65-0.75)")
    
    args = parser.parse_args()
    
    if args.mode == 'build':
        build_codebook(args.bone_dir, args.output)
    elif args.mode == 'detect':
        # Derive paths from image name if provided
        if args.image_name:
            base_dir = "/home/kneepolean/sreeharsha/bone_data"
            args.cluster_stats = f"{base_dir}/dinov3_vitb16/{args.image_name}/bone_features.json"
            args.image = f"{base_dir}/jeans/{args.image_name}.png"
            
            # Check if files exist
            if not os.path.exists(args.cluster_stats):
                print(f"ERROR: Cluster stats not found: {args.cluster_stats}")
                return
            if not os.path.exists(args.image):
                print(f"ERROR: Image not found: {args.image}")
                return
        else:
            # Using --cluster-stats, so --image is required
            if args.image is None:
                detect_parser.error("--image is required when using --cluster-stats")
        
        detect_bone(args.codebook, args.cluster_stats, args.image, 
                   args.output_dir, args.min_size, args.threshold)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()

