//
//  ContentView.swift
//  SVD_ROS_Comms
//
//  Unified app with tabbed interface: Control Panel + Drill Site Overlay
//

import SwiftUI

struct ContentView: View {
    @Environment(AppModel.self) private var appModel
    
    // Three TCP clients for three ports
    @StateObject private var controlClient = TCPClient(host: "192.168.0.193", port: 5000)
    @StateObject private var poseClient = DrillPosesTCPClient(host: "192.168.0.193", port: 5001)
    @StateObject private var imageClient = ImageTCPClient(host: "192.168.0.193", port: 5002)
    
    var body: some View {
        TabView {
            // Tab 1: Control Panel (existing SVD_ROS_Comms UI)
            ControlPanelView(
                controlClient: controlClient,
                imageClient: imageClient,
                poseClient: poseClient
            )
            .environment(appModel)
            .tabItem {
                Label("Control Panel", systemImage: "slider.horizontal.3")
            }
            
            // Tab 2: Drill Sites Visualization (ArucoTransform UI)
            DrillSiteVisualizationView(
                poseClient: poseClient
            )
            .environment(appModel)
            .tabItem {
                Label("Overlay", systemImage: "cube.transparent")
            }
        }
        .onAppear {
            setupClients()
        }
    }
    
    private func setupClients() {
        // Setup FSM state callback for control client
        controlClient.onFsmStateReceived = { [appModel] newState in
            print("📍 ContentView: FSM state callback received: '\(newState)'")
            Task { @MainActor in
                // Update last state before changing current
                if appModel.fsmState != newState && !appModel.fsmState.isEmpty && appModel.fsmState != "unknown" {
                    appModel.lastFsmState = appModel.fsmState
                }
                appModel.fsmState = newState
                print("📍 ContentView: Updated appModel.fsmState to: '\(appModel.fsmState)'")
            }
        }
        
        // Setup drill poses callback
        poseClient.onDrillPosesReceived = { [appModel] drillSites in
            Task { @MainActor in
                appModel.drillSites = drillSites
            }
        }
    }
}

#Preview {
    ContentView()
}
