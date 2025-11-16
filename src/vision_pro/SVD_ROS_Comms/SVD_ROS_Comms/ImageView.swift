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
    @Environment(\.dismiss) private var dismiss
    let onAnnotationsComplete: ([AnnotationPoint]) -> Void
    
    private let maxAnnotations = 2
    
    var body: some View {
        VStack(spacing: 20) {
            // Header with close button
            HStack {
                Text("Tap on the image to annotate (2 points required)")
                    .font(.headline)
                    .foregroundColor(.white)
                
                Spacer()
                
                Button {
                    dismiss()
                } label: {
                    Image(systemName: "xmark.circle.fill")
                        .font(.system(size: 28))
                        .foregroundColor(.white)
                }
            }
            .padding(.horizontal)
            
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
                                    .frame(width: 10, height: 10)
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
                Text("Annotations: \(annotations.count)/\(maxAnnotations)")
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
                Button("Undo Last") {
                    if !annotations.isEmpty {
                        annotations.removeLast()
                    }
                }
                .padding()
                .background(Color.orange)
                .foregroundColor(.white)
                .cornerRadius(8)
                .disabled(annotations.isEmpty)
                
                Button("Clear All") {
                    annotations.removeAll()
                }
                .padding()
                .background(Color.red)
                .foregroundColor(.white)
                .cornerRadius(8)
                .disabled(annotations.isEmpty)
                
                Button("Send Annotations") {
                    onAnnotationsComplete(annotations)
                    dismiss()
                }
                .padding()
                .background(annotations.count == maxAnnotations ? Color.green : Color.gray)
                .foregroundColor(.white)
                .cornerRadius(8)
                .disabled(annotations.count != maxAnnotations)
            }
        }
        .padding()
        .background(Color.gray.opacity(0.1))
        .cornerRadius(15)
    }
    
    private func addAnnotation(at location: CGPoint, in viewSize: CGSize) {
        // Only allow up to maxAnnotations
        guard annotations.count < maxAnnotations else {
            return
        }
        
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
