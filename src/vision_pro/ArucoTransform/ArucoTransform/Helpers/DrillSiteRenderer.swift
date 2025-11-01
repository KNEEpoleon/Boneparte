//
//  DrillSiteRenderer.swift
//  ArucoTransform
//
//  Renders drill sites as red spheres with axis vectors
//

import Foundation
import RealityKit
import SwiftUI

@MainActor
class DrillSiteRenderer: ObservableObject {
    let rootEntity = Entity()
    
    private var drillSiteEntities: [UUID: Entity] = [:]
    
    // Visual configuration
    private let sphereRadius: Float = 0.01  // 1cm radius
    private let axisLength: Float = 0.05    // 5cm axis length
    private let axisThickness: Float = 0.002  // 2mm thickness
    
    func updateDrillSites(_ drillSites: [DrillSite], arucoTransform: simd_float4x4?) {
        guard let arucoToWorld = arucoTransform else {
            // No transform yet, hide all
            clearDrillSites()
            return
        }
        
        // Remove drill sites that no longer exist
        let currentIDs = Set(drillSites.map { $0.id })
        let existingIDs = Set(drillSiteEntities.keys)
        let toRemove = existingIDs.subtracting(currentIDs)
        
        for id in toRemove {
            if let entity = drillSiteEntities[id] {
                entity.removeFromParent()
                drillSiteEntities.removeValue(forKey: id)
            }
        }
        
        // Add or update drill sites
        for drillSite in drillSites {
            if let existingEntity = drillSiteEntities[drillSite.id] {
                // Update existing
                updateDrillSiteEntity(existingEntity, drillSite: drillSite, arucoToWorld: arucoToWorld)
            } else {
                // Create new
                let entity = createDrillSiteEntity(drillSite: drillSite, arucoToWorld: arucoToWorld)
                rootEntity.addChild(entity)
                drillSiteEntities[drillSite.id] = entity
            }
        }
        
        print("🎯 Rendered \(drillSites.count) drill sites")
    }
    
    private func createDrillSiteEntity(drillSite: DrillSite, arucoToWorld: simd_float4x4) -> Entity {
        let container = Entity()
        container.name = "DrillSite_\(drillSite.id)"
        
        // Create red sphere at drill site location
        let sphere = ModelEntity(
            mesh: .generateSphere(radius: sphereRadius),
            materials: [SimpleMaterial(color: .red, isMetallic: false)]
        )
        sphere.name = "Sphere"
        container.addChild(sphere)
        
        // Create axis vectors (X, Y, Z) to show orientation
        // Z-axis (drilling direction) - blue and longer
        let zAxis = createAxisCylinder(
            length: axisLength * 2,
            color: .blue,
            direction: SIMD3<Float>(0, 0, 1)
        )
        zAxis.name = "ZAxis"
        container.addChild(zAxis)
        
        // X-axis - red
        let xAxis = createAxisCylinder(
            length: axisLength,
            color: UIColor.red.withAlphaComponent(0.7),
            direction: SIMD3<Float>(1, 0, 0)
        )
        xAxis.name = "XAxis"
        container.addChild(xAxis)
        
        // Y-axis - green
        let yAxis = createAxisCylinder(
            length: axisLength,
            color: UIColor.green.withAlphaComponent(0.7),
            direction: SIMD3<Float>(0, 1, 0)
        )
        yAxis.name = "YAxis"
        container.addChild(yAxis)
        
        // Set position and orientation
        updateDrillSiteEntity(container, drillSite: drillSite, arucoToWorld: arucoToWorld)
        
        return container
    }
    
    private func createAxisCylinder(length: Float, color: UIColor, direction: SIMD3<Float>) -> ModelEntity {
        let cylinder = ModelEntity(
            mesh: .generateCylinder(height: length, radius: axisThickness),
            materials: [SimpleMaterial(color: color, isMetallic: false)]
        )
        
        // Position cylinder to start at origin and extend in direction
        cylinder.position = direction * (length / 2)
        
        // Rotate cylinder to align with direction
        // Default cylinder is along Y-axis, need to rotate to match direction
        if direction.z != 0 {
            // Z-axis: rotate 90° around X
            cylinder.orientation = simd_quatf(angle: .pi / 2, axis: SIMD3<Float>(1, 0, 0))
        } else if direction.x != 0 {
            // X-axis: rotate 90° around Z
            cylinder.orientation = simd_quatf(angle: .pi / 2, axis: SIMD3<Float>(0, 0, 1))
        }
        // Y-axis: no rotation needed (default)
        
        return cylinder
    }
    
    private func updateDrillSiteEntity(_ entity: Entity, drillSite: DrillSite, arucoToWorld: simd_float4x4) {
        // Drill site is in ArUco marker frame, transform to world
        let drillSiteInAruco = simd_float4x4(
            drillSite.orientation,
            SIMD3<Float>(drillSite.position.x, drillSite.position.y, drillSite.position.z)
        )
        
        let drillSiteInWorld = arucoToWorld * drillSiteInAruco
        
        // Extract position and orientation
        entity.position = SIMD3<Float>(
            drillSiteInWorld.columns.3.x,
            drillSiteInWorld.columns.3.y,
            drillSiteInWorld.columns.3.z
        )
        
        entity.orientation = simd_quatf(drillSiteInWorld)
    }
    
    private func clearDrillSites() {
        for (_, entity) in drillSiteEntities {
            entity.removeFromParent()
        }
        drillSiteEntities.removeAll()
    }
}

// MARK: - Helper Extensions

extension simd_float4x4 {
    init(_ quaternion: simd_quatf, _ translation: SIMD3<Float>) {
        let rotation = simd_float3x3(quaternion)
        self.init(
            SIMD4<Float>(rotation.columns.0, 0),
            SIMD4<Float>(rotation.columns.1, 0),
            SIMD4<Float>(rotation.columns.2, 0),
            SIMD4<Float>(translation, 1)
        )
    }
}

extension simd_quatf {
    init(_ matrix: simd_float4x4) {
        let rotation = simd_float3x3(
            matrix.columns.0.xyz,
            matrix.columns.1.xyz,
            matrix.columns.2.xyz
        )
        self.init(rotation)
    }
}

extension SIMD4 where Scalar == Float {
    var xyz: SIMD3<Float> {
        return SIMD3<Float>(x, y, z)
    }
}

