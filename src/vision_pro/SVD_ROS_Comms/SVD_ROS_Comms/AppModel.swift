//
//  AppModel.swift
//  SVD_ROS_Comms
//
//  Created by Kneepoleon Boneparte on 4/16/25.
//

import SwiftUI

/// Maintains app-wide state
@MainActor
@Observable
class AppModel {
    let immersiveSpaceID = "ImmersiveSpace"
    enum ImmersiveSpaceState {
        case closed
        case inTransition
        case open
    }
    var immersiveSpaceState = ImmersiveSpaceState.closed
    
    // Image annotation state
    enum AnnotationState {
        case idle
        case waitingForImage
        case imageReceived
        case annotating
        case sendingAnnotations
        case complete
    }
    var annotationState = AnnotationState.idle
    var currentImageData: Data?
    var currentAnnotations: [AnnotationPoint] = []
    
    // NEW: Drill site visualization state (from ArucoTransform)
    var isTransformEnabled = false
    var arucoTransformEstablished = false
    var drillSites: [DrillSite] = []
    
    // NEW: FSM state from ROS
    var fsmState: String = "unknown"
    var lastFsmState: String = "unknown"  // For "Last State" display
}

// DrillSite struct (from ArucoTransform)
struct DrillSite: Identifiable {
    let id = UUID()
    let position: SIMD3<Float>
    let orientation: simd_quatf
    
    init(position: SIMD3<Float>, orientation: simd_quatf) {
        self.position = position
        self.orientation = orientation
    }
}
