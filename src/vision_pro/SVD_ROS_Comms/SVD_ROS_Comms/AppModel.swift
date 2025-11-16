//
//  AppModel.swift
//  SVD_ROS_Comms
//
//  Created by Kneepoleon Boneparte on 4/16/25.
//

import SwiftUI

/// Maintains app-wide state
@MainActor
class AppModel: ObservableObject {
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
    @Published var annotationState = AnnotationState.idle
    @Published var currentImageData: Data?
    @Published var currentAnnotations: [AnnotationPoint] = []
}
