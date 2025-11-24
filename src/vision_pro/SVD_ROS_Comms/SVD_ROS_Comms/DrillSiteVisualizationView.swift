//
//  DrillSiteVisualizationView.swift
//  SVD_ROS_Comms
//
//  Drill Site Visualization Tab (Tab 2) - Redesigned with BONEparte Design System
//

import SwiftUI
import RealityKit

struct DrillSiteVisualizationView: View {
    @ObservedObject var poseClient: DrillPosesTCPClient
    @Environment(AppModel.self) private var appModel
    @Environment(\.openImmersiveSpace) var openImmersiveSpace
    @Environment(\.dismissImmersiveSpace) var dismissImmersiveSpace
    
    // Connection status
    private var isConnected: Bool {
        poseClient.isConnected
    }
    
    var body: some View {
        VStack(alignment: .leading, spacing: 0) {
            // TOP BANNER: Branding and Connection Status
            connectionBanner
            
            // Main content
            ScrollView {
                VStack(alignment: .leading, spacing: Spacing.lg) {
                    // Connection status card
                    connectionStatusCard
                    
                    // Visualization controls
                    visualizationControlCard
                    
                    // ArUco detection status (when immersive space is open)
                    if appModel.immersiveSpaceState == .open {
                        arucoStatusCard
                    }
                }
                .padding(Spacing.xl)
            }
        }
        .background(GlassmorphismBackground())
        .onAppear {
            // Setup drill pose callback
            poseClient.onDrillPosesReceived = { (drillSites: [DrillSite]) in
                appModel.drillSites = drillSites
            }
        }
    }
    
    // MARK: - Connection Banner (Top)
    
    private var connectionBanner: some View {
        HStack(spacing: Spacing.md) {
            // BONEparte branding (LEFT)
            VStack(alignment: .leading, spacing: 2) {
                Text("BONEparte")
                    .font(.system(size: 28, weight: .bold))
                    .foregroundColor(Color(red: 1.0, green: 0.60, blue: 0.20))
                
                Text("Drill Site Overlay")
                    .font(.system(size: 10))
                    .foregroundColor(.gray)
            }
            
            Spacer()
            
            // Status indicator (RIGHT)
            HStack(spacing: Spacing.sm) {
                Circle()
                    .fill(isConnected ? Color.statusActive : Color.statusDanger)
                    .frame(width: 12, height: 12)
                    .shadow(color: isConnected ? Color.statusActive : Color.statusDanger,
                           radius: 4)
                
                Text(isConnected ? "ONLINE" : "OFFLINE")
                    .font(.labelMedium)
                    .foregroundColor(isConnected ? .statusActive : .statusDanger)
                    .fontWeight(.bold)
            }
            
            Text("Poses: 192.168.0.193:5001")
                .font(.caption)
                .foregroundColor(.textSecondary)
            
            // Connect/Disconnect button
            Button(action: {
                if isConnected {
                    poseClient.disconnect()
                    appModel.drillSites = []
                } else {
                    poseClient.connect()
                }
            }) {
                Text(isConnected ? "Disconnect" : "Connect")
                    .font(.labelSmall)
                    .foregroundColor(isConnected ? .statusDanger : .statusActive)
            }
            .buttonStyle(.bordered)
        }
        .padding(.horizontal, Spacing.lg)
        .padding(.vertical, Spacing.md)
        .background(Color.surfaceElevated)
    }
    
    // MARK: - Connection Status Card
    
    private var connectionStatusCard: some View {
        StatusCard(
            title: "Connection Status",
            status: isConnected ? .active : .danger,
            statusText: poseClient.statusMessage
        ) {
            VStack(spacing: Spacing.md) {
                DataRow(
                    label: "Server",
                    value: "192.168.0.193:5001",
                    isMonospaced: true
                )
                
                DataRow(
                    label: "Drill Poses Received",
                    value: "\(appModel.drillSites.count)",
                    valueColor: appModel.drillSites.count > 0 ? .statusActive : .textSecondary
                )
                
                DataRow(
                    label: "Connection State",
                    value: isConnected ? "Connected" : "Disconnected",
                    valueColor: isConnected ? .statusActive : .statusDanger
                )
            }
        }
    }
    
    // MARK: - Visualization Control Card
    
    private var visualizationControlCard: some View {
        VStack(spacing: Spacing.md) {
            SectionHeader(
                "3D Visualization",
                subtitle: "ArUco-based spatial overlay",
                icon: "cube.transparent"
            )
            
            StandardCard {
                VStack(spacing: Spacing.lg) {
                    // Info text
                    if !isConnected {
                        HStack(spacing: Spacing.sm) {
                            Image(systemName: "info.circle.fill")
                                .foregroundColor(.statusWarning)
                            Text("Please connect to drill poses server to enable visualization")
                                .font(.bodySmall)
                                .foregroundColor(.textSecondary)
                        }
                        .padding(Spacing.sm)
                        .background(
                            RoundedRectangle(cornerRadius: Spacing.cornerSmall)
                                .fill(Color.statusWarning.opacity(0.1))
                        )
                    }
                    
                    // Show/Hide Drill Sites button
                    Button {
                        Task {
                            if appModel.immersiveSpaceState == .open {
                                await dismissImmersiveSpace()
                                appModel.immersiveSpaceState = .closed
                                appModel.isTransformEnabled = false
                                appModel.arucoTransformEstablished = false
                                appModel.worldAnchorManager.reset()
                            } else {
                                // Reset transform before opening to force fresh ArUco detection
                                // This ensures the transform is correct for the current world coordinate system
                                appModel.arucoTransformEstablished = false
                                appModel.worldAnchorManager.reset()
                                appModel.immersiveSpaceState = .inTransition
                                await openImmersiveSpace(id: appModel.immersiveSpaceID)
                                appModel.immersiveSpaceState = .open
                                appModel.isTransformEnabled = true
                            }
                        }
                    } label: {
                        Label(
                            appModel.immersiveSpaceState == .open ? "Hide Drill Sites" : "Show Drill Sites",
                            systemImage: appModel.immersiveSpaceState == .open ? "eye.slash.fill" : "eye.fill"
                        )
                    }
                    .primaryButton(fullWidth: true)
                    .disabled(!isConnected || appModel.immersiveSpaceState == .inTransition)
                    .opacity((!isConnected || appModel.immersiveSpaceState == .inTransition) ? 0.5 : 1.0)
                    
                    // Instructions
                    VStack(alignment: .leading, spacing: Spacing.sm) {
                        HStack(spacing: Spacing.xs) {
                            Image(systemName: "1.circle.fill")
                                .foregroundColor(.surgicalBlue)
                            Text("Ensure ArUco marker (ID=0, 17cm) is visible")
                                .font(.caption)
                                .foregroundColor(.textSecondary)
                        }
                        
                        HStack(spacing: Spacing.xs) {
                            Image(systemName: "2.circle.fill")
                                .foregroundColor(.surgicalBlue)
                            Text("Point Vision Pro at the marker near the robot.")
                                .font(.caption)
                                .foregroundColor(.textSecondary)
                        }
                        
                        HStack(spacing: Spacing.xs) {
                            Image(systemName: "3.circle.fill")
                                .foregroundColor(.surgicalBlue)
                            Text("Drill sites will appear once marker is detected")
                                .font(.caption)
                                .foregroundColor(.textSecondary)
                        }
                    }
                    .padding(Spacing.md)
                    .background(
                        RoundedRectangle(cornerRadius: Spacing.cornerSmall)
                            .fill(Color.surfaceLight)
                    )
                }
            }
        }
    }
    
    // MARK: - ArUco Detection Status Card
    
    private var arucoStatusCard: some View {
        StandardCard {
            VStack(spacing: Spacing.md) {
                if appModel.arucoTransformEstablished {
                    // Marker detected
                    HStack(spacing: Spacing.md) {
                        Image(systemName: "checkmark.seal.fill")
                            .font(.system(size: Spacing.iconLarge))
                            .foregroundColor(.statusActive)
                        
                        VStack(alignment: .leading, spacing: Spacing.xs) {
                            Text("ArUco Marker Detected")
                                .font(.headlineMedium)
                                .foregroundColor(.textPrimary)
                            
                            Text("Transform established and locked")
                                .font(.bodySmall)
                                .foregroundColor(.textSecondary)
                        }
                        
                        Spacer()
                        
                        // Drill sites count badge
                        VStack(spacing: Spacing.xs) {
                            Text("\(appModel.drillSites.count)")
                                .font(.system(size: 28, weight: .bold))
                                .foregroundColor(.surgicalBlue)
                            
                            Text("Sites")
                                .font(.caption)
                                .foregroundColor(.textSecondary)
                        }
                        .padding(Spacing.md)
                        .background(
                            RoundedRectangle(cornerRadius: Spacing.cornerSmall)
                                .fill(Color.surgicalBlue.opacity(0.1))
                        )
                    }
                    
                    Divider()
                    
                    DataRow(
                        label: "Marker ID",
                        value: "0",
                        isMonospaced: true
                    )
                    
                    DataRow(
                        label: "Marker Size",
                        value: "15 cm",
                        isMonospaced: true
                    )
                    
                    DataRow(
                        label: "Visualization Active",
                        value: "Yes",
                        valueColor: .statusActive
                    )
                    
                } else {
                    // Searching for marker
                    HStack(spacing: Spacing.md) {
                        ProgressView()
                            .scaleEffect(1.2)
                            .tint(.surgicalBlue)
                        
                        VStack(alignment: .leading, spacing: Spacing.xs) {
                            Text("Searching for ArUco Marker")
                                .font(.headlineMedium)
                                .foregroundColor(.textPrimary)
                            
                            Text("Point your Vision Pro at the marker")
                                .font(.bodySmall)
                                .foregroundColor(.textSecondary)
                        }
                        
                        Spacer()
                    }
                    .padding(Spacing.md)
                    .background(
                        RoundedRectangle(cornerRadius: Spacing.cornerSmall)
                            .fill(Color.statusProcessing.opacity(0.1))
                    )
                }
            }
        }
        .asCard(title: "ArUco Detection Status")
    }
}
