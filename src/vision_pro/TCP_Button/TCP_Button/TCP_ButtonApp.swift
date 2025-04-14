//
//  TCP_ButtonApp.swift
//  TCP_Button
//
//  Created by Kneepoleon Boneparte on 4/10/25.
//

import SwiftUI

@main
struct TCP_ButtonApp: App {
    @State private var appModel = AppModel()
    @State private var trackingAppState = AppState()  // <-- ADD THIS

    var body: some Scene {
        WindowGroup {
            ContentView()    // <-- PASS IT HERE
                .environment(appModel)
        }

        ImmersiveSpace(id: appModel.immersiveSpaceID) {
            ImmersiveView()
                .environment(appModel)
                .onAppear {
                    appModel.immersiveSpaceState = .open
                }
                .onDisappear {
                    appModel.immersiveSpaceState = .closed
                }
        }
        .immersionStyle(selection: .constant(.full), in: .full)

        ImmersiveSpace(id: "ObjectTracking") {    // <-- ALSO ADD TRACKING IMMERSIVE SPACE
            ObjectTrackingRealityView(appState: trackingAppState)
        }
    }
}

