//
//  SVD_ROS_CommsApp.swift
//  SVD_ROS_Comms
//
//  Created by Kneepoleon Boneparte on 4/16/25.
//

import SwiftUI

@main
struct SVD_ROS_CommsApp: App {

    @State private var appModel = AppModel()

    var body: some Scene {
        WindowGroup {
            ContentView()
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
    }
}
