//
//  ImageView.swift
//  SVD_ROS_Comms
//
//  Created by Kneepoleon Boneparte on 4/16/25.
//

import SwiftUI

struct ImageView: View {
    let imageData: Data
    @State private var annotations: [AnnotationPoint] = []
    @State private var imageSize: CGSize = .zero
    let onAnnotationsComplete: ([AnnotationPoint]) -> Void
    
    var body: some View {
        VStack(spacing: 20) {
            Text("Tap on the image to annotate")
                .font(.headline)
                .foregroundColor(.white)
            
            // Image with tap gesture
            GeometryReader { geometry in
                if let uiImage = UIImage(data: imageData) {
                    Image(uiImage: uiImage)
                        .resizable()
                        .aspectRatio(contentMode: .fit)
                        .background(Color.black)
                        .onTapGesture { location in
                            addAnnotation(at: location, in: geometry.size)
                        }
                        .overlay(
                            // Draw annotation points
                            ForEach(Array(annotations.enumerated()), id: \.offset) { index, annotation in
                                Circle()
                                    .fill(Color.red)
                                    .frame(width: 20, height: 20)
                                    .position(
                                        x: CGFloat(annotation.x) * geometry.size.width,
                                        y: CGFloat(annotation.y) * geometry.size.height
                                    )
                            }
                        )
                        .onAppear {
                            imageSize = geometry.size
                        }
                } else {
                    Text("Failed to load image")
                        .foregroundColor(.red)
                        .frame(maxWidth: .infinity, maxHeight: .infinity)
                        .background(Color.black)
                }
            }
            .frame(maxHeight: 400)
            .background(Color.black)
            .cornerRadius(10)
            
            // Annotation info
            VStack(spacing: 10) {
                Text("Annotations: \(annotations.count)")
                    .font(.subheadline)
                    .foregroundColor(.white)
                
                if !annotations.isEmpty {
                    ScrollView(.horizontal, showsIndicators: false) {
                        HStack(spacing: 10) {
                            ForEach(Array(annotations.enumerated()), id: \.offset) { index, annotation in
                                Text("P\(index + 1): (\(String(format: "%.2f", annotation.x)), \(String(format: "%.2f", annotation.y)))")
                                    .font(.caption)
                                    .padding(5)
                                    .background(Color.blue.opacity(0.7))
                                    .foregroundColor(.white)
                                    .cornerRadius(5)
                            }
                        }
                        .padding(.horizontal)
                    }
                }
            }
            
            // Action buttons
            HStack(spacing: 20) {
                Button("Clear All") {
                    annotations.removeAll()
                }
                .padding()
                .background(Color.orange)
                .foregroundColor(.white)
                .cornerRadius(8)
                .disabled(annotations.isEmpty)
                
                Button("Send Annotations") {
                    onAnnotationsComplete(annotations)
                }
                .padding()
                .background(Color.green)
                .foregroundColor(.white)
                .cornerRadius(8)
                .disabled(annotations.isEmpty)
            }
        }
        .padding()
        .background(Color.gray.opacity(0.1))
        .cornerRadius(15)
    }
    
    private func addAnnotation(at location: CGPoint, in viewSize: CGSize) {
        // Convert tap location to normalized coordinates (0.0 to 1.0)
        let normalizedX = max(0.0, min(1.0, Double(location.x / viewSize.width)))
        let normalizedY = max(0.0, min(1.0, Double(location.y / viewSize.height)))
        
        let annotation = AnnotationPoint(x: normalizedX, y: normalizedY)
        annotations.append(annotation)
    }
}

#Preview {
    let sampleImage = UIImage(systemName: "photo")?.pngData() ?? Data()
    
    return ImageView(
        imageData: sampleImage,
        onAnnotationsComplete: { _ in }
    )
    .background(Color.black)
}
