//
//  Spacing.swift
//  BONEparte Surgical Suite
//
//  Design System: Spacing & Layout
//  Consistent spacing scale for professional medical UI
//

import SwiftUI

/// Spacing constants following 8pt grid system
struct Spacing {
    
    // MARK: - Base Spacing Scale (8pt grid)
    
    /// Extra small - 4pt (tight spacing)
    static let xs: CGFloat = 4
    
    /// Small - 8pt (compact spacing)
    static let sm: CGFloat = 8
    
    /// Medium - 16pt (standard spacing)
    static let md: CGFloat = 16
    
    /// Large - 24pt (comfortable spacing)
    static let lg: CGFloat = 24
    
    /// Extra large - 32pt (spacious)
    static let xl: CGFloat = 32
    
    /// 2x Extra large - 48pt (very spacious)
    static let xxl: CGFloat = 48
    
    /// 3x Extra large - 64pt (maximum spacing)
    static let xxxl: CGFloat = 64
    
    
    // MARK: - Component-Specific Spacing
    
    /// Padding inside cards and panels
    static let cardPadding: CGFloat = 20
    
    /// Spacing between cards in a stack
    static let cardSpacing: CGFloat = 16
    
    /// Padding inside buttons
    static let buttonPadding: CGFloat = 16
    
    /// Spacing between button groups
    static let buttonGroupSpacing: CGFloat = 12
    
    /// Padding for form fields
    static let fieldPadding: CGFloat = 12
    
    /// Spacing between form fields
    static let fieldSpacing: CGFloat = 16
    
    /// Section header spacing
    static let sectionSpacing: CGFloat = 24
    
    
    // MARK: - Layout Margins
    
    /// Window edge margins
    static let windowMargin: CGFloat = 30
    
    /// Container edge margins
    static let containerMargin: CGFloat = 20
    
    /// Content inset margins
    static let contentInset: CGFloat = 16
    
    
    // MARK: - Icon Sizes
    
    /// Small icon - 16pt
    static let iconSmall: CGFloat = 16
    
    /// Medium icon - 24pt
    static let iconMedium: CGFloat = 24
    
    /// Large icon - 32pt
    static let iconLarge: CGFloat = 32
    
    /// Extra large icon - 48pt
    static let iconXLarge: CGFloat = 48
    
    
    // MARK: - Border & Corner Radius
    
    /// Subtle border width
    static let borderThin: CGFloat = 1
    
    /// Standard border width
    static let borderMedium: CGFloat = 2
    
    /// Thick border width
    static let borderThick: CGFloat = 3
    
    /// Small corner radius - subtle rounding
    static let cornerSmall: CGFloat = 8
    
    /// Medium corner radius - standard cards
    static let cornerMedium: CGFloat = 12
    
    /// Large corner radius - prominent elements
    static let cornerLarge: CGFloat = 16
    
    /// Extra large corner radius - special elements
    static let cornerXLarge: CGFloat = 24
    
    
    // MARK: - Component Heights
    
    /// Minimum touchable height (accessibility)
    static let minTouchTarget: CGFloat = 44
    
    /// Small button height
    static let buttonSmall: CGFloat = 36
    
    /// Standard button height
    static let buttonMedium: CGFloat = 44
    
    /// Large button height
    static let buttonLarge: CGFloat = 52
    
    /// Text field height
    static let textFieldHeight: CGFloat = 44
    
    /// Status bar height
    static let statusBarHeight: CGFloat = 32
    
    /// Navigation bar height
    static let navBarHeight: CGFloat = 56
    
    
    // MARK: - Shadow Styles
    
    /// Subtle shadow - for slight elevation
    static let shadowSubtle = ShadowStyle(
        color: Color.black.opacity(0.08),
        radius: 4,
        x: 0,
        y: 2
    )
    
    /// Medium shadow - for cards
    static let shadowMedium = ShadowStyle(
        color: Color.black.opacity(0.12),
        radius: 8,
        x: 0,
        y: 4
    )
    
    /// Large shadow - for elevated panels
    static let shadowLarge = ShadowStyle(
        color: Color.black.opacity(0.16),
        radius: 16,
        x: 0,
        y: 8
    )
}


// MARK: - Shadow Style

struct ShadowStyle {
    let color: Color
    let radius: CGFloat
    let x: CGFloat
    let y: CGFloat
}


// MARK: - Edge Insets Presets

extension EdgeInsets {
    
    /// Zero insets
    static let zero = EdgeInsets(top: 0, leading: 0, bottom: 0, trailing: 0)
    
    /// Small uniform insets
    static let small = EdgeInsets(
        top: Spacing.sm,
        leading: Spacing.sm,
        bottom: Spacing.sm,
        trailing: Spacing.sm
    )
    
    /// Medium uniform insets
    static let medium = EdgeInsets(
        top: Spacing.md,
        leading: Spacing.md,
        bottom: Spacing.md,
        trailing: Spacing.md
    )
    
    /// Large uniform insets
    static let large = EdgeInsets(
        top: Spacing.lg,
        leading: Spacing.lg,
        bottom: Spacing.lg,
        trailing: Spacing.lg
    )
    
    /// Card insets
    static let card = EdgeInsets(
        top: Spacing.cardPadding,
        leading: Spacing.cardPadding,
        bottom: Spacing.cardPadding,
        trailing: Spacing.cardPadding
    )
    
    /// Horizontal only insets
    static func horizontal(_ value: CGFloat) -> EdgeInsets {
        EdgeInsets(top: 0, leading: value, bottom: 0, trailing: value)
    }
    
    /// Vertical only insets
    static func vertical(_ value: CGFloat) -> EdgeInsets {
        EdgeInsets(top: value, leading: 0, bottom: value, trailing: 0)
    }
}


// MARK: - View Extensions

extension View {
    
    /// Apply card padding
    func cardPadding() -> some View {
        self.padding(Spacing.cardPadding)
    }
    
    /// Apply content inset
    func contentInset() -> some View {
        self.padding(Spacing.contentInset)
    }
    
    /// Apply window margin
    func windowMargin() -> some View {
        self.padding(Spacing.windowMargin)
    }
    
    /// Apply subtle shadow
    func shadowSubtle() -> some View {
        let s = Spacing.shadowSubtle
        return self.shadow(color: s.color, radius: s.radius, x: s.x, y: s.y)
    }
    
    /// Apply medium shadow
    func shadowMedium() -> some View {
        let s = Spacing.shadowMedium
        return self.shadow(color: s.color, radius: s.radius, x: s.x, y: s.y)
    }
    
    /// Apply large shadow
    func shadowLarge() -> some View {
        let s = Spacing.shadowLarge
        return self.shadow(color: s.color, radius: s.radius, x: s.x, y: s.y)
    }
}


// MARK: - Layout Presets

struct Layout {
    
    /// Small window size (compact controls)
    static let windowSmall = CGSize(width: 400, height: 500)
    
    /// Medium window size (standard panels)
    static let windowMedium = CGSize(width: 600, height: 700)
    
    /// Large window size (dashboards)
    static let windowLarge = CGSize(width: 800, height: 900)
    
    /// Extra large window size (full monitoring)
    static let windowXLarge = CGSize(width: 1000, height: 1000)
    
    
    /// Minimum button width for consistency
    static let minButtonWidth: CGFloat = 100
    
    /// Maximum content width for readability
    static let maxContentWidth: CGFloat = 800
    
    /// Grid column count for multi-column layouts
    static let gridColumns = 3
    
    /// Grid item minimum size
    static let gridItemMinSize: CGFloat = 120
}

