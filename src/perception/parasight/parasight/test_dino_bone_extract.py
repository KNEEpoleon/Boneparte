from dino_bone_extract import DINOBoneExtractor

def test_dino_bone_extract():
    image_path = "/home/kneepolean/sreeharsha/bone_data/jeans/bottom_0pin_1_Color.png"
    checkpoint_path = "/checkpoints/dinov3_vitb16_pretrain_lvd1689m-73cec8be.pth"
    output_dir = "/home/kneepolean/sreeharsha/bone_data/testing_boneparte_integration"
    dinov3_location = "/home/kneepolean/sreeharsha/dinov3"
    extractor = DINOBoneExtractor(checkpoint_path=checkpoint_path, dinov3_location=dinov3_location, output_home=output_dir)
    centroid = extractor.get_centroid(image_path)
    print(centroid)

if __name__ == "__main__":
    test_dino_bone_extract()