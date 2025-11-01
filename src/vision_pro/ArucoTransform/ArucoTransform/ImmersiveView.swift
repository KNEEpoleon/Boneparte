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
    @State private var imageTracking: ImageTrackingProvider?
    
    @State private var worldAnchorManager = WorldAnchorManager()
    @State private var drillSiteRenderer = DrillSiteRenderer()
    
    var body: some View {
        RealityView { content in
            // Add drill site renderer root entity
            content.add(drillSiteRenderer.rootEntity)
            
            // Start ARKit session with image tracking
            Task {
                await startARKitSession()
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
        .overlay(alignment: .top) {
            VStack(spacing: 15) {
                if !appModel.arucoTransformEstablished {
                    // ArUco detection status
                    VStack(spacing: 8) {
                        ProgressView()
                            .scaleEffect(1.2)
                        Text("🔍 Looking for ArUco Marker")
                            .font(.title2)
                            .fontWeight(.bold)
                        Text("Point your Vision Pro at the 15cm ArUco marker (ID=0)")
                            .font(.body)
                            .foregroundStyle(.secondary)
                        Text("The marker should be mounted on the robot base")
                            .font(.caption)
                            .foregroundStyle(.tertiary)
                    }
                    .padding(25)
                    .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 20))
                } else {
                    // ArUco detected - show drill sites info
                    VStack(spacing: 8) {
                        HStack {
                            Image(systemName: "checkmark.circle.fill")
                                .foregroundStyle(.green)
                                .font(.title2)
                            Text("ArUco Marker Locked")
                                .font(.title3)
                                .fontWeight(.semibold)
                        }
                        Text("Displaying \(appModel.drillSites.count) drill site(s)")
                            .font(.body)
                            .foregroundStyle(.secondary)
                    }
                    .padding(20)
                    .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 15))
                }
            }
            .padding(.top, 50)
        }
    }
    
    private func startARKitSession() async {
        do {
            // Load reference images from AR Resources group (same as working FemurTracking2 app)
            let referenceImages = ReferenceImage.loadReferenceImages(inGroupNamed: "AR Resources", bundle: nil)
            
            print("✅ Loaded \(referenceImages.count) reference image(s) for tracking")
            
            guard !referenceImages.isEmpty else {
                print("❌ No reference images found in 'AR Resources' group")
                try await arKitSession.run([worldTracking])
                print("⚠️ ARKit session started with world tracking only")
                return
            }
            
            // Create image tracking provider
            imageTracking = ImageTrackingProvider(referenceImages: referenceImages)
            
            // Start ARKit session with both world tracking and image tracking
            try await arKitSession.run([worldTracking, imageTracking!])
            print("✅ ARKit session started with world and image tracking")
            
        } catch {
            print("❌ Failed to start ARKit session: \(error)")
        }
    }
    
    private func detectArucoMarker() async {
        // Only detect once if not already detected
        guard !appModel.arucoTransformEstablished else { return }
        
        guard let imageTracking = imageTracking else {
            print("❌ Image tracking provider not available")
            return
        }
        
        print("🔍 Waiting for ArUco marker detection...")
        
        // Monitor image anchor updates
        for await anchorUpdate in imageTracking.anchorUpdates {
            let anchor = anchorUpdate.anchor
            
            guard anchorUpdate.event == .added || anchorUpdate.event == .updated else {
                continue
            }
            
            guard anchor.isTracked else {
                print("⚠️ ArUco marker detected but not tracked")
                continue
            }
            
            print("✅ ArUco marker '\(anchor.referenceImage.name ?? "unknown")' detected and tracked!")
            
            // Get the transform from the image anchor
            // anchor.originFromAnchorTransform gives us ArUco marker → World transform
            let arucoToWorld = anchor.originFromAnchorTransform
            
            // Store the transform
            await MainActor.run {
                worldAnchorManager.setArucoTransform(arucoToWorld)
                appModel.arucoTransformEstablished = true
                appModel.isArucoDetected = true
            }
            
            print("✅ Transform established: ArUco → World")
            print("  Position: (\(arucoToWorld.columns.3.x), \(arucoToWorld.columns.3.y), \(arucoToWorld.columns.3.z))")
            
            // Only need to detect once
            break
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

