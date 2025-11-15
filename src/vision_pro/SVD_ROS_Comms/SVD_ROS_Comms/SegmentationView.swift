//
//  SegmentationView.swift
//  SVD_ROS_Comms
//
//  Segmentation Review View - Accept or Reject segmentation results
//

import SwiftUI

struct SegmentationView: View {
    let imageData: Data
    @Environment(\.dismiss) private var dismiss
    let onAccept: () -> Void
    let onReject: () -> Void
    @State private var showTimeout = false
    
    var body: some View {
        VStack(spacing: 20) {
            // Header (no close button - must accept or reject)
            HStack {
                Text("Review Segmentation Result")
                    .font(.headline)
                    .foregroundColor(.white)
                
                Spacer()
            }
            .padding(.horizontal)
            
            // Display segmented image
            GeometryReader { geometry in
                if let uiImage = UIImage(data: imageData) {
                    Image(uiImage: uiImage)
                        .resizable()
                        .aspectRatio(contentMode: .fit)
                        .background(Color.black)
                } else {
                    Text("Failed to load segmented image")
                        .foregroundColor(.red)
                        .frame(maxWidth: .infinity, maxHeight: .infinity)
                        .background(Color.black)
                }
            }
            .frame(maxHeight: 400)
            .background(Color.black)
            .cornerRadius(10)
            
            // Instructions
            Text("Review the segmentation overlay. Accept to continue or Reject to re-annotate.")
                .font(.subheadline)
                .foregroundColor(.textSecondary)
                .multilineTextAlignment(.center)
                .padding(.horizontal)
            
            // Action buttons
            HStack(spacing: 20) {
                Button("Reject") {
                    onReject()
                }
                .padding()
                .frame(minWidth: 120)
                .background(Color.statusDanger)
                .foregroundColor(.white)
                .cornerRadius(8)
                .font(.headline)
                
                Button("Accept") {
                    onAccept()
                }
                .padding()
                .frame(minWidth: 120)
                .background(Color.statusActive)
                .foregroundColor(.white)
                .cornerRadius(8)
                .font(.headline)
            }
            .padding(.bottom)
        }
        .padding()
        .background(Color.gray.opacity(0.1))
        .cornerRadius(15)
    }
}

#Preview {
    let sampleImage = UIImage(systemName: "photo")?.pngData() ?? Data()
    
    return SegmentationView(
        imageData: sampleImage,
        onAccept: { },
        onReject: { }
    )
    .background(Color.black)
}
