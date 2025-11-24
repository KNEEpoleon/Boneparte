//
//  ControlPanelView.swift
//  SVD_ROS_Comms
//
//  Main Control Panel View - Tab 1
//

import SwiftUI

struct ControlPanelView: View {
    @ObservedObject var controlClient: TCPClient
    @ObservedObject var imageClient: ImageTCPClient
    @ObservedObject var poseClient: DrillPosesTCPClient
    @Environment(AppModel.self) private var appModel
    
    @State private var selectedDrillSite: String? = nil
    @State private var showEmergencyConfirm = false
    @State private var showClearSiteConfirm = false
    @State private var drillCommandHistory: [DrillCommand] = []
    @State private var lastConnectionState: Bool = false
    @State private var storedOriginalImage: Data? = nil
    @State private var showingAnnotation: Bool = false  // Track if annotation sheet is currently showing
    @State private var showingSegmentation: Bool = false  // Track if segmentation sheet is currently showing
    
    // Command mapping (updated for 4 drill sites: 2 femur, 2 tibia)
    private let commandMap: [String: String] = [
        "Proceed Mission": "proceed_mission",
        "Reset Mission": "reset_mission",
        "Annotate": "annotate",
        "robot_away": "robot_away",
        "robot_home": "robot_home",
        "Femur 1": "drill_femur_1",
        "Femur 2": "drill_femur_2",
        "Tibia 1": "drill_femur_3",  // Old femur_3 is now tibia_1
        "Tibia 2": "drill_tibia_1"   // Old tibia_1 is now tibia_2
    ]
    
    // Connection status
    private var isConnected: Bool {
        controlClient.statusColor == .green
    }
    
    var body: some View {
        VStack(alignment: .leading, spacing: 0) {
            // Main content
            HStack(alignment: .top, spacing: Spacing.xl) {
                leftColumn
                    .frame(width: 420, alignment: .topLeading)
                rightColumn
                    .frame(width: 540, alignment: .topLeading)
            }
            .padding(Spacing.xl)
            
            Spacer(minLength: 0)
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
        .background(GlassmorphismBackground())
        .alert("Emergency Stop", isPresented: $showEmergencyConfirm) {
            Button("Cancel", role: .cancel) { }
            Button("STOP ALL", role: .destructive) {
                controlClient.send("KILLALL\n")
            }
        } message: {
            Text("This will immediately stop all robotic operations. Continue?")
        }
        .alert("Clear Drill Site", isPresented: $showClearSiteConfirm) {
            Button("Cancel", role: .cancel) { }
            
            // Femur options
            Button("Femur 1") {
                sendClearCommand("clear_femur_1")
            }
            Button("Femur 2") {
                sendClearCommand("clear_femur_2")
            }
            
            // Tibia options
            Button("Tibia 1") {
                sendClearCommand("clear_femur_3")
            }
            Button("Tibia 2") {
                sendClearCommand("clear_tibia_1")
            }
        } message: {
            Text("Select which drill site obstacle to clear from the planning stack:")
        }
        .sheet(isPresented: $showingAnnotation) {
            if let annotationImageData = imageClient.receivedImage {
                ImageView(imageData: annotationImageData) { annotations in
                    // Store original image before sending annotations
                    storedOriginalImage = annotationImageData
                    imageClient.sendAnnotations(annotations)
                    imageClient.receivedImage = nil
                    showingAnnotation = false
                }
            }
        }
        .sheet(isPresented: $showingSegmentation) {
            if let segmentedImageData = imageClient.receivedSegmentedImage {
                SegmentationView(imageData: segmentedImageData) {
                    // Accept callback
                    controlClient.send("accept\n")
                    storedOriginalImage = nil
                    imageClient.receivedSegmentedImage = nil
                    showingSegmentation = false
                } onReject: {
                    // Reject callback - send reject and reopen annotation window
                    controlClient.send("reject\n")
                    imageClient.receivedSegmentedImage = nil
                    showingSegmentation = false
                    // Reopen annotation window with stored original image
                    if let originalImage = storedOriginalImage {
                        DispatchQueue.main.asyncAfter(deadline: .now() + 0.3) {
                            imageClient.receivedImage = originalImage
                        }
                    }
                }
            }
        }
        .onChange(of: isConnected) { oldValue, newValue in
            // Clear history when transitioning from disconnected to connected
            if !lastConnectionState && newValue {
                drillCommandHistory.removeAll()
            }
            lastConnectionState = newValue
        }
        .onChange(of: imageClient.receivedImage) { oldValue, newValue in
            // When annotation image arrives, show the sheet
            if newValue != nil && !showingAnnotation {
                showingAnnotation = true
            }
        }
        .onChange(of: imageClient.receivedSegmentedImage) { oldValue, newValue in
            // When segmented image arrives, show the sheet
            if newValue != nil && !showingSegmentation {
                showingSegmentation = true
            }
        }
    }
    
    // MARK: - Connection Banner (Top)
    
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
    
    private var leftColumn: some View {
        VStack(alignment: .leading, spacing: Spacing.lg) {
            connectionBanner
            workflowControlPanel
        }
    }
    
    private var rightColumn: some View {
        VStack(alignment: .leading, spacing: Spacing.lg) {
            emergencySection
            clearSiteSection
            drillSiteSelection
        }
    }
    
    // MARK: - Workflow Control Panel
    
    private var workflowControlPanel: some View {
        VStack(spacing: Spacing.md) {
            SectionHeader(
                "Control Panel",
                subtitle: "Core surgical workflow",
                icon: "slider.horizontal.3"
            )
            
            StandardCard {
                Button {
                    sendCommand("Proceed Mission")
                } label: {
                    Label("Proceed Mission", systemImage: "play.circle.fill")
                }
                .primaryButton(fullWidth: true)
            }
            
            StandardCard {
                Button {
                    sendCommand("Reset Mission")
                } label: {
                    Label("Reset Mission", systemImage: "arrow.counterclockwise.circle.fill")
                }
                .primaryButton(fullWidth: true)
            }
            
            StandardCard {
                Button {
                    sendCommand("Annotate")
                } label: {
                    Label("Annotate", systemImage: "pencil.tip.crop.circle")
                }
                .primaryButton(fullWidth: true)
            }
            
            StandardCard {
                Button {
                    sendCommand("robot_away")
                } label: {
                    Label("Send Robot Away", systemImage: "paperplane.fill")
                }
                .primaryButton(fullWidth: true)
            }
            
            StandardCard {
                Button {
                    sendCommand("robot_home")
                } label: {
                    Label("Bring Robot Home", systemImage: "house.fill")
                }
                .primaryButton(fullWidth: true)
            }
        }
    }
    
    // MARK: - Drill Site Selection
    
    private var drillSiteSelection: some View {
        VStack(spacing: Spacing.md) {
            SectionHeader(
                "Drill Site Selection",
                subtitle: "Select target drill location",
                icon: "target"
            )
            
            StandardCard(title: "Target Sites") {
                VStack(spacing: Spacing.lg) {
                    // Femur sites
                    VStack(alignment: .leading, spacing: Spacing.sm) {
                        HStack {
                            Image(systemName: "circle.fill")
                                .foregroundColor(.femurColor)
                                .font(.system(size: 10))
                            Text("Femur")
                                .font(.labelLarge)
                                .foregroundColor(.textPrimary)
                        }
                        
                        HStack(spacing: Spacing.sm) {
                            ForEach(1...2, id: \.self) { index in
                                Button {
                                    selectDrillSite("Femur \(index)")
                                } label: {
                                    drillSiteButton(
                                        number: index,
                                        color: .femurColor,
                                        isSelected: selectedDrillSite == "Femur \(index)"
                                    )
                                }
                            }
                        }
                    }
                    
                    Divider()
                    
                    // Tibia sites
                    VStack(alignment: .leading, spacing: Spacing.sm) {
                        HStack {
                            Image(systemName: "circle.fill")
                                .foregroundColor(.tibiaColor)
                                .font(.system(size: 10))
                            Text("Tibia")
                                .font(.labelLarge)
                                .foregroundColor(.textPrimary)
                        }
                        
                        HStack(spacing: Spacing.sm) {
                            ForEach(1...2, id: \.self) { index in
                                Button {
                                    selectDrillSite("Tibia \(index)")
                                } label: {
                                    drillSiteButton(
                                        number: index,
                                        color: .tibiaColor,
                                        isSelected: selectedDrillSite == "Tibia \(index)"
                                    )
                                }
                            }
                        }
                    }
                    
                    // Selected site info
                    if let selected = selectedDrillSite {
                        Divider()
                        
                        HStack {
                            Image(systemName: "checkmark.circle.fill")
                                .foregroundColor(.statusActive)
                            Text("Selected: \(selected)")
                                .font(.labelMedium)
                                .foregroundColor(.textPrimary)
                            
                            Spacer()
                            
                            Button("Clear") {
                                selectedDrillSite = nil
                            }
                            .font(.labelSmall)
                            .foregroundColor(.surgicalBlue)
                        }
                    }
                    
                    // Drill command history
                    if !drillCommandHistory.isEmpty {
                        Divider()
                        
                        VStack(alignment: .leading, spacing: Spacing.sm) {
                            HStack {
                                Text("Drill History")
                                    .font(.labelLarge)
                                    .foregroundColor(.textPrimary)
                                
                                Spacer()
                                
                                Button("Clear All") {
                                    drillCommandHistory.removeAll()
                                }
                                .font(.labelSmall)
                                .foregroundColor(.statusDanger)
                            }
                            
                            ScrollView {
                                VStack(spacing: Spacing.xs) {
                                    ForEach(drillCommandHistory) { command in
                                        HStack {
                                            Circle()
                                                .fill(command.siteName.contains("Femur") ? Color.femurColor : Color.tibiaColor)
                                                .frame(width: 8, height: 8)
                                            
                                            Text(command.siteName)
                                                .font(.caption)
                                                .foregroundColor(.textPrimary)
                                            
                                            Spacer()
                                            
                                            Text(command.timestamp, style: .time)
                                                .font(.caption2)
                                                .foregroundColor(.textSecondary)
                                        }
                                        .padding(.vertical, 4)
                                    }
                                }
                            }
                            .frame(maxHeight: 150)
                        }
                    }
                }
            }
        }
    }
    
    // MARK: - Emergency Section
    
    private var emergencySection: some View {
        VStack(spacing: Spacing.md) {
            Button {
                showEmergencyConfirm = true
            } label: {
                HStack(spacing: Spacing.md) {
                    Image(systemName: "hand.raised.fill")
                        .font(.system(size: Spacing.iconLarge))
                    Text("EMERGENCY STOP")
                        .font(.buttonLarge)
                }
            }
            .emergencyButton()
            
            Text("Immediately halts all robotic motion")
                .font(.caption)
                .foregroundColor(.textSecondary)
        }
    }
    
    // MARK: - Clear Site Section
    
    private var clearSiteSection: some View {
        Button {
            showClearSiteConfirm = true
        } label: {
            HStack(spacing: Spacing.md) {
                Image(systemName: "xmark.circle.fill")
                    .font(.system(size: 20))
                Text("Clear Drill Site")
                    .font(.buttonLarge)
            }
            .frame(maxWidth: .infinity)
            .padding(.vertical, Spacing.md)
        }
        .buttonStyle(.borderedProminent)
        .tint(Color(red: 1.0, green: 0.60, blue: 0.20))
        .controlSize(.large)
    }
    
    // MARK: - Helper Views
    
    private func drillSiteButton(number: Int, color: Color, isSelected: Bool) -> some View {
        VStack(spacing: Spacing.xs) {
            ZStack {
                Circle()
                    .fill(isSelected ? color : Color.surfaceElevated)
                    .frame(width: 50, height: 50)
                
                Circle()
                    .stroke(color, lineWidth: isSelected ? 3 : 2)
                    .frame(width: 50, height: 50)
                
                Text("\(number)")
                    .font(.headlineMedium)
                    .fontWeight(.bold)
                    .foregroundColor(isSelected ? .white : color)
            }
        }
        .scaleEffect(isSelected ? 1.1 : 1.0)
        .animation(.spring(response: 0.3), value: isSelected)
    }
    
    // MARK: - Helper Methods
    
    private func sendCommand(_ label: String) {
        guard let command = commandMap[label] else { return }
        controlClient.send("\(command)\n")
    }
    
    private func sendClearCommand(_ command: String) {
        controlClient.send("\(command)\n")
        print("📤 Sent clear command: \(command)")
    }
    
    private func selectDrillSite(_ site: String) {
        selectedDrillSite = site
        sendCommand(site)
        
        // Add to history
        let command = DrillCommand(siteName: site, timestamp: Date())
        drillCommandHistory.append(command)
    }
}

// MARK: - Supporting Views from original ContentView

struct GlassmorphismBackground: View {
    var body: some View {
        ZStack {
            // Deep charcoal base
            Color.backgroundPrimary
            
            // Subtle gradient for depth (black to dark gray)
            LinearGradient(
                colors: [
                    Color.black.opacity(0.3),
                    Color(red: 0.15, green: 0.15, blue: 0.16).opacity(0.2)
                ],
                startPoint: .topLeading,
                endPoint: .bottomTrailing
            )
        }
        .ignoresSafeArea()
    }
}

// Helper struct to make Data Identifiable for sheet presentation
struct ImageData: Identifiable {
    let id = UUID()
    let data: Data
}

// Drill command tracking
struct DrillCommand: Identifiable {
    let id = UUID()
    let siteName: String
    let timestamp: Date
}

