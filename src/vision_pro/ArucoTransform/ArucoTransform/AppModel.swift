//
//  AppModel.swift
//  ArucoTransform
//
//  Application state management
//

import SwiftUI

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
    
    // TCP Connection state
    var isConnected = false
    var connectionStatus = "Disconnected"
    
    // Transform state
    var isTransformEnabled = false
    var isArucoDetected = false
    var arucoTransformEstablished = false
    
    // Drill site data
    var drillSites: [DrillSite] = []
    
    // Server configuration
    var serverIP = "192.168.0.193"  // Static IP for ROS server
    var serverPort: UInt16 = 5001
}

struct DrillSite: Identifiable {
    let id = UUID()
    let position: SIMD3<Float>
    let orientation: simd_quatf
    
    init(position: SIMD3<Float>, orientation: simd_quatf) {
        self.position = position
        self.orientation = orientation
    }
}

