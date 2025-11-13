from dino_bone_extract import DINOBoneExtractor

def test_dino_bone_extract():
    # image_path = "/home/kneepolean/sreeharsha/bone_data/jeans/top_0pin_3_Color.png"
    # checkpoint_path = "/checkpoints/dinov3_vitb16_pretrain_lvd1689m-73cec8be.pth"
    # output_dir = "/home/kneepolean/sreeharsha/bone_data/testing_boneparte_integration"
    # dinov3_path = "/home/kneepolean/sreeharsha/dinov3"
    # codebook_path = "/home/kneepolean/sreeharsha/bone_data/extracted_features_dataset/fvd_bone_codebook.json"
    
    image_path = "/ros_ws/src/perception/auto_reposition/center_0pin_1_Color/center_0pin_1_Color.png"
    checkpoint_path = "/ros_ws/src/perception/dinov3/checkpoints/dinov3_vitb16_pretrain_lvd1689m-73cec8be.pth"
    output_dir = "/ros_ws/src/perception/auto_reposition/"
    dinov3_path = "/ros_ws/src/perception/dinov3"
    codebook_path = "/ros_ws/src/perception/auto_reposition/fvd_bone_codebook.json"
    
    extractor = DINOBoneExtractor(
        checkpoint_path=checkpoint_path,
        dinov3_path=dinov3_path,
        codebook_path=codebook_path,
        output_home=output_dir
    )
    
    centroid = extractor.get_centroid(image_path)
    print(f"\nFinal result:")
    print(centroid.keys())
    # import pdb; pdb.set_trace()

if __name__ == "__main__":
    test_dino_bone_extract()