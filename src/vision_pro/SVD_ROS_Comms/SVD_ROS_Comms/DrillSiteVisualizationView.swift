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
    @ObservedObject var controlClient: TCPClient
    @ObservedObject var imageClient: ImageTCPClient
    @Environment(AppModel.self) private var appModel
    @Environment(\.openImmersiveSpace) var openImmersiveSpace
    @Environment(\.dismissImmersiveSpace) var dismissImmersiveSpace
    
    // Connection status
    private var isConnected: Bool {
        controlClient.isConnected && poseClient.isConnected && imageClient.isConnected
    }
    
    var body: some View {
        VStack(alignment: .leading, spacing: 0) {
            // Main content
            ScrollView {
                VStack(alignment: .leading, spacing: Spacing.lg) {
                    // Connection banner (moved from top)
                    connectionBanner
                    
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
    
    // MARK: - Connection Banner
    
    private var connectionBanner: some View {
        VStack(alignment: .leading, spacing: Spacing.sm) {
            // First line: BONEparte branding
            HStack(spacing: Spacing.xs) {
                Text("BONEparte")
                    .font(.system(size: 28, weight: .bold))
                    .foregroundColor(Color(red: 1.0, green: 0.60, blue: 0.20))
                
                Text("Surgical Control Suite")
                    .font(.system(size: 14))
                    .foregroundColor(.gray)
            }
            
            // Second line: Status, server info, and connect button
            HStack(spacing: Spacing.sm) {
                // Status indicator
                HStack(spacing: Spacing.xs) {
                    Circle()
                        .fill(isConnected ? Color.statusActive : Color.statusDanger)
                        .frame(width: 10, height: 10)
                        .shadow(color: isConnected ? Color.statusActive : Color.statusDanger,
                               radius: 3)
                    
                    Text(isConnected ? "ONLINE" : "OFFLINE")
                        .font(.caption)
                        .foregroundColor(isConnected ? .statusActive : .statusDanger)
                        .fontWeight(.semibold)
                }
                
                Text("ROS Server: 192.168.0.193")
                    .font(.caption)
                    .foregroundColor(.textSecondary)
                
                Spacer()
                
                // Connect/Disconnect button
                Button(action: {
                    if isConnected {
                        controlClient.disconnect()
                        poseClient.disconnect()
                        imageClient.disconnect()
                    } else {
                        controlClient.connect()
                        poseClient.connect()
                        imageClient.connect()
                    }
                }) {
                    Text(isConnected ? "Disconnect" : "Connect")
                        .font(.caption)
                        .foregroundColor(isConnected ? .statusDanger : .statusActive)
                }
                .buttonStyle(.bordered)
            }
        }
        .padding(.horizontal, Spacing.lg)
        .padding(.vertical, Spacing.md)
        .background(Color.surfaceElevated)
        .cornerRadius(Spacing.cornerMedium)
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
