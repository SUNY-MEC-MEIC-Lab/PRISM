import open3d as o3d
import numpy as np
import os
import argparse
import glob
import time

def load_point_cloud(file_path):
    return o3d.io.read_point_cloud(file_path)

def prism_sampling(pcd, k=1, quantization=1.0, use_chromaticity=True):
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)  # [0, 1] float
    
    if len(points) == 0:
        return pcd

    # 1. Color Transformation
    if use_chromaticity:
        sum_colors = np.sum(colors, axis=1, keepdims=True) + 1e-6
        normalized_colors = colors / sum_colors
        colors_to_bin = normalized_colors * 255.0
    else:
        colors_to_bin = colors * 255.0

    # 2. Quantization
    colors_quant = (colors_to_bin / quantization).round().astype(np.int32)
    
    # 3. Efficient Grouping via Sorting
    dtype = np.dtype((np.void, colors_quant.dtype.itemsize * colors_quant.shape[1]))
    colors_packed = np.ascontiguousarray(colors_quant).view(dtype)
    
    # Shuffle indices first to ensure random selection within bins
    N = len(points)
    shuffled_indices = np.arange(N)
    np.random.shuffle(shuffled_indices)
    
    # Apply shuffle to packed colors
    colors_packed_shuffled = colors_packed[shuffled_indices]
    
    # Sort the shuffled colors to group identical bins together
    sort_order = np.argsort(colors_packed_shuffled.ravel())
    sorted_colors = colors_packed_shuffled[sort_order]
    
    # Find unique groups and their counts
    _, unique_indices, unique_counts = np.unique(sorted_colors, return_index=True, return_counts=True)
    
    # 4. Stratified Selection
    group_starts_expanded = np.repeat(unique_indices, unique_counts)
    
    # Create a counter for each element within its group
    current_indices = np.arange(len(sorted_colors))
    intra_group_counter = current_indices - group_starts_expanded
    
    # Keep only the first k elements of each group
    keep_mask_in_sorted = intra_group_counter < k
    
    # 5. Map back to original indices
    final_original_indices = shuffled_indices[sort_order[keep_mask_in_sorted]]
    
    # Create new PointCloud
    limited_pcd = o3d.geometry.PointCloud()
    limited_pcd.points = o3d.utility.Vector3dVector(points[final_original_indices])
    limited_pcd.colors = o3d.utility.Vector3dVector(colors[final_original_indices])
    
    if pcd.has_normals():
        normals = np.asarray(pcd.normals)
        limited_pcd.normals = o3d.utility.Vector3dVector(normals[final_original_indices])
        
    return limited_pcd

def process_file(input_path, output_path, k, quantization, use_chromaticity):
    start_time = time.time()
    
    pcd = load_point_cloud(input_path)
    
    if not pcd.has_colors():
        print(f"[WARN] Skipping {os.path.basename(input_path)}: No color data found.")
        return
        
    original_points = len(pcd.points)
    
    clean_pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=100, std_ratio=2.0)
    
    # Apply PRISM
    reduced_pcd = prism_sampling(clean_pcd, k=k, quantization=quantization, use_chromaticity=use_chromaticity)
    
    # Save result
    o3d.io.write_point_cloud(output_path, reduced_pcd)
    
    end_time = time.time()
    duration = end_time - start_time
    final_points = len(reduced_pcd.points)
    
    chroma_tag = "Chroma" if use_chromaticity else "RGB"
    print(f"[PRISM] {os.path.basename(input_path)} | {chroma_tag}-Q{quantization}-k{k} | T={duration:.2f}s | {original_points} -> {final_points} pts")

def main():
    parser = argparse.ArgumentParser(description="PRISM: Color-Stratified Point Cloud Sampling")
    parser.add_argument("--input", "-i", type=str, required=True, help="Input file or directory (PLY format)")
    parser.add_argument("--output", "-o", type=str, required=True, help="Output file or directory")
    
    # PRISM Parameters
    parser.add_argument("--k", "-k", type=int, default=1, help="Bin capacity (max points per color bin)")
    parser.add_argument("--quantization", "-q", type=float, default=1.0, help="Color quantization step size (default: 1.0)")
    parser.add_argument("--no_chromaticity", action="store_true", help="Disable chromaticity normalization (use raw RGB)")
    
    args = parser.parse_args()
    
    # Invert the flag for logic
    use_chromaticity = not args.no_chromaticity
    
    if os.path.isfile(args.input):
        process_file(args.input, args.output, args.k, args.quantization, use_chromaticity)
    elif os.path.isdir(args.input):
        if not os.path.exists(args.output):
            os.makedirs(args.output)
            
        ply_files = glob.glob(os.path.join(args.input, "**/*.ply"), recursive=True)
        print(f"Found {len(ply_files)} PLY files in {args.input}")
        
        for input_path in ply_files:
            rel_path = os.path.relpath(input_path, args.input)
            output_path = os.path.join(args.output, rel_path)
            
            output_dir = os.path.dirname(output_path)
            if not os.path.exists(output_dir):
                os.makedirs(output_dir)
                
            process_file(input_path, output_path, args.k, args.quantization, use_chromaticity)
    else:
        print(f"Error: Input path {args.input} not found.")

if __name__ == "__main__":
    main()
