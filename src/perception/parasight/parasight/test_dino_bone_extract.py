from dino_bone_extract import DINOBoneExtractor

def test_dino_bone_extract():
    image_path = "/home/kneepolean/sreeharsha/bone_data/jeans/top_0pin_3_Color.png"
    checkpoint_path = "/checkpoints/dinov3_vitb16_pretrain_lvd1689m-73cec8be.pth"
    output_dir = "/home/kneepolean/sreeharsha/bone_data/testing_boneparte_integration"
    dinov3_path = "/home/kneepolean/sreeharsha/dinov3"
    codebook_path = "/home/kneepolean/sreeharsha/bone_data/bone_codebook.pkl"
    extractor = DINOBoneExtractor(checkpoint_path=checkpoint_path, dinov3_path=dinov3_path, codebook_path=codebook_path, output_home=output_dir)
    
    
    centroid = extractor.get_centroid(image_path)
    print(centroid)

if __name__ == "__main__":
    test_dino_bone_extract()