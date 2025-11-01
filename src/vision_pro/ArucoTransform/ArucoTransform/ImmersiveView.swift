//
//  ImmersiveView.swift
//  ArucoTransform
//
//  Immersive AR view for ArUco detection and drill site rendering
//

import SwiftUI
import RealityKit
import ARKit

struct ImmersiveView: View {
    @Environment(AppModel.self) private var appModel
    
    @State private var arKitSession = ARKitSession()
    @State private var worldTracking = WorldTrackingProvider()
    @State private var cameraFrameProvider = CameraFrameProvider()
    
    @State private var arucoDetector = ArucoDetector()
    @State private var worldAnchorManager = WorldAnchorManager()
    @State private var drillSiteRenderer = DrillSiteRenderer()
    
    var body: some View {
        RealityView { content in
            // Add drill site renderer root entity
            content.add(drillSiteRenderer.rootEntity)
            
            // Start ARKit session
            Task {
                do {
                    try await arKitSession.run([worldTracking, cameraFrameProvider])
                    print("ARKit session started successfully")
                } catch {
                    print("Failed to start ARKit session: \(error)")
                }
            }
            
            // Start ArUco detection loop
            Task {
                await detectArucoMarker()
            }
            
            // Start render loop
            Task {
                await renderLoop()
            }
        } update: { content in
            // Update drill sites when they change
            Task { @MainActor in
                drillSiteRenderer.updateDrillSites(
                    appModel.drillSites,
                    arucoTransform: worldAnchorManager.arucoToWorldTransform
                )
            }
        }
    }
    
    private func detectArucoMarker() async {
        // Only detect once if not already detected
        guard !appModel.arucoTransformEstablished else { return }
        
        for await cameraFrame in cameraFrameProvider.cameraFrameUpdates(for: arKitSession) {
            guard let pixelBuffer = cameraFrame.sample.pixelBuffer else { continue }
            
            // Detect ArUco marker
            if let markerTransform = arucoDetector.detectMarker(
                in: pixelBuffer,
                intrinsics: cameraFrame.sample.intrinsics
            ) {
                // Get device transform in world space
                guard let deviceAnchor = worldTracking.queryDeviceAnchor(
                    atTimestamp: cameraFrame.sample.time
                ) else { continue }
                
                let deviceToWorld = deviceAnchor.originFromAnchorTransform
                
                // Camera is the device for Vision Pro
                let cameraToWorld = deviceToWorld
                
                // Compute ArUco to world transform
                // aruco_to_world = camera_to_world * camera_to_aruco
                let arucoToWorld = cameraToWorld * markerTransform
                
                // Store the transform
                await MainActor.run {
                    worldAnchorManager.arucoToWorldTransform = arucoToWorld
                    appModel.arucoTransformEstablished = true
                    appModel.isArucoDetected = true
                }
                
                print("✅ ArUco marker detected and localized!")
                print("Transform established: ArUco → World")
                
                // Only detect once
                break
            }
        }
    }
    
    private func renderLoop() async {
        // Update drill sites rendering at 30 Hz
        while appModel.isTransformEnabled {
            try? await Task.sleep(for: .milliseconds(33))
            
            // Drill sites are automatically updated via RealityView's update closure
        }
    }
}

#Preview(immersionStyle: .mixed) {
    ImmersiveView()
        .environment(AppModel())
}

