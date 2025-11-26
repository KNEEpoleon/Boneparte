//
//  WorldAnchorManager.swift
//  ArucoTransform
//
//  Manages world anchors and coordinate transforms
//

import Foundation
import ARKit

@MainActor
class WorldAnchorManager: ObservableObject {
    // Transform from ArUco marker to world space
    @Published var arucoToWorldTransform: simd_float4x4 = matrix_identity_float4x4
    
    // Whether the anchor is established
    @Published var isAnchorEstablished = false
    
    // Optional: Persist anchor for future sessions
    private var worldAnchor: WorldAnchor?
    
    func setArucoTransform(_ transform: simd_float4x4) {
        self.arucoToWorldTransform = transform
        self.isAnchorEstablished = true
        
        print("World anchor established at:")
        print("  Position: \(transform.columns.3.x), \(transform.columns.3.y), \(transform.columns.3.z)")
    }
    
    func transformFromArucoToWorld(_ arucoPosition: SIMD3<Float>) -> SIMD3<Float> {
        // Transform a point from ArUco marker space to world space
        let homogeneous = SIMD4<Float>(arucoPosition.x, arucoPosition.y, arucoPosition.z, 1.0)
        let worldHomogeneous = arucoToWorldTransform * homogeneous
        return SIMD3<Float>(worldHomogeneous.x, worldHomogeneous.y, worldHomogeneous.z)
    }
    
    func transformPoseFromArucoToWorld(_ arucoTransform: simd_float4x4) -> simd_float4x4 {
        // Transform a full pose (position + orientation) from ArUco space to world space
        return arucoToWorldTransform * arucoTransform
    }
    
    func reset() {
        arucoToWorldTransform = matrix_identity_float4x4
        isAnchorEstablished = false
        worldAnchor = nil
        print("World anchor reset")
    }
}

