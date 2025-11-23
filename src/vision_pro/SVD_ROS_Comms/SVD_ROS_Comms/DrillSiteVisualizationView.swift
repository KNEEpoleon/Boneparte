//
//  DrillSiteVisualizationView.swift
//  SVD_ROS_Comms
//
//  Drill Site Visualization Tab (Tab 2) - Based on ArucoTransform
//

import SwiftUI
import RealityKit

struct DrillSiteVisualizationView: View {
    @ObservedObject var poseClient: DrillPosesTCPClient
    @Environment(AppModel.self) private var appModel
    @Environment(\.openImmersiveSpace) var openImmersiveSpace
    @Environment(\.dismissImmersiveSpace) var dismissImmersiveSpace
    
    var body: some View {
        VStack(spacing: 20) {
            Text("Drill Site Overlay")
                .font(.title)
                .fontWeight(.bold)
            
            Divider()
            
            // Connection section
            VStack(alignment: .leading, spacing: 10) {
                Text("Drill Poses Connection")
                    .font(.headline)
                
                // Display static IP/Port
                HStack {
                    Text("Server:")
                    Text("192.168.0.193:5001")
                        .foregroundColor(.secondary)
                        .fontWeight(.medium)
                }
                
                Button(action: {
                    if poseClient.isConnected {
                        poseClient.disconnect()
                        appModel.drillSites = []
                    } else {
                        poseClient.connect()
                    }
                }) {
                    HStack {
                        Image(systemName: poseClient.isConnected ? "wifi.circle.fill" : "wifi.slash")
                        Text(poseClient.isConnected ? "Disconnect" : "Connect")
                    }
                    .frame(maxWidth: .infinity)
                }
                .buttonStyle(.borderedProminent)
                .tint(poseClient.isConnected ? .red : .blue)
                
                Text(poseClient.statusMessage)
                    .font(.caption)
                    .foregroundColor(poseClient.statusColor)
            }
            .padding()
            .background(Color.secondary.opacity(0.1))
            .cornerRadius(10)
            
            Divider()
            
            // Transform section
            VStack(alignment: .leading, spacing: 10) {
                Text("3D Visualization")
                    .font(.headline)
                
                if !poseClient.isConnected {
                    Text("Please connect to drill poses server first")
                        .foregroundColor(.secondary)
                        .font(.caption)
                }
                
                // Show Drill Sites button
                Button(action: {
                    Task {
                        if appModel.immersiveSpaceState == .open {
                            await dismissImmersiveSpace()
                            appModel.immersiveSpaceState = .closed
                            appModel.isTransformEnabled = false
                            appModel.arucoTransformEstablished = false
                        } else {
                            // Clear old drill sites and transform before opening new immersive space
                            appModel.drillSites = []
                            appModel.arucoTransformEstablished = false
                            appModel.immersiveSpaceState = .inTransition
                            await openImmersiveSpace(id: appModel.immersiveSpaceID)
                            appModel.immersiveSpaceState = .open
                            appModel.isTransformEnabled = true
                        }
                    }
                }) {
                    HStack {
                        Image(systemName: appModel.immersiveSpaceState == .open ? "eye.fill" : "eye.slash")
                        Text(appModel.immersiveSpaceState == .open ? "Hide Drill Sites" : "Show Drill Sites")
                    }
                    .frame(maxWidth: .infinity)
                }
                .buttonStyle(.borderedProminent)
                .tint(appModel.immersiveSpaceState == .open ? .orange : .green)
                .disabled(!poseClient.isConnected || appModel.immersiveSpaceState == .inTransition)
                
                if appModel.immersiveSpaceState == .open {
                    if appModel.arucoTransformEstablished {
                        HStack {
                            Image(systemName: "checkmark.circle.fill")
                                .foregroundColor(.green)
                            Text("ArUco marker detected & localized")
                                .font(.caption)
                        }
                        
                        Text("Drill sites: \(appModel.drillSites.count)")
                            .font(.caption)
                            .foregroundColor(.secondary)
                    } else {
                        HStack {
                            ProgressView()
                                .scaleEffect(0.7)
                            Text("Looking for ArUco marker (ID=0)...")
                                .font(.caption)
                        }
                        
                        Text("Point your Vision Pro at the 15cm ArUco marker")
                            .font(.caption2)
                            .foregroundColor(.secondary)
                    }
                }
            }
            .padding()
            .background(Color.secondary.opacity(0.1))
            .cornerRadius(10)
            
            Spacer()
            
            // Status footer
            VStack(spacing: 5) {
                if poseClient.isConnected && appModel.isTransformEnabled {
                    Text("System Active")
                        .font(.caption)
                        .foregroundColor(.green)
                } else {
                    Text("System Idle")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
            }
        }
        .padding(30)
        .frame(width: 400, height: 550)
        .onAppear {
            // Setup drill pose callback
            poseClient.onDrillPosesReceived = { (drillSites: [DrillSite]) in
                appModel.drillSites = drillSites
            }
        }
    }
}

