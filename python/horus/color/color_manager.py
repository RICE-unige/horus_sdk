"""
Color assignment system for robot visualizations in HORUS SDK
"""

import random
import hashlib
from typing import Dict, Tuple, List, Optional
from dataclasses import dataclass, field
from enum import Enum


class ColorScheme(Enum):
    """Predefined color schemes for robot visualizations"""
    BRIGHT = "bright"
    PASTEL = "pastel"
    DARK = "dark"
    RAINBOW = "rainbow"
    NEON = "neon"


@dataclass
class RGBColor:
    """RGB color representation"""
    r: int  # 0-255
    g: int  # 0-255
    b: int  # 0-255
    a: float = 1.0  # Alpha channel 0.0-1.0
    
    def __post_init__(self):
        """Validate RGB values"""
        for value, name in [(self.r, 'red'), (self.g, 'green'), (self.b, 'blue')]:
            if not 0 <= value <= 255:
                raise ValueError(f"{name} value must be between 0 and 255")
        if not 0.0 <= self.a <= 1.0:
            raise ValueError("Alpha value must be between 0.0 and 1.0")
    
    def to_hex(self) -> str:
        """Convert to hex string (#RRGGBB)"""
        return f"#{self.r:02x}{self.g:02x}{self.b:02x}"
    
    def to_rgba_tuple(self) -> Tuple[int, int, int, float]:
        """Convert to RGBA tuple"""
        return (self.r, self.g, self.b, self.a)
    
    def to_normalized_tuple(self) -> Tuple[float, float, float, float]:
        """Convert to normalized RGBA tuple (0.0-1.0)"""
        return (self.r / 255.0, self.g / 255.0, self.b / 255.0, self.a)
    
    def __str__(self) -> str:
        return self.to_hex()
    
    @classmethod
    def from_hex(cls, hex_color: str, alpha: float = 1.0) -> 'RGBColor':
        """Create RGBColor from hex string"""
        hex_color = hex_color.lstrip('#')
        if len(hex_color) != 6:
            raise ValueError("Hex color must be 6 characters")
        
        r = int(hex_color[0:2], 16)
        g = int(hex_color[2:4], 16)
        b = int(hex_color[4:6], 16)
        
        return cls(r, g, b, alpha)
    
    @classmethod
    def from_hsv(cls, h: float, s: float, v: float, alpha: float = 1.0) -> 'RGBColor':
        """Create RGBColor from HSV values (h: 0-360, s,v: 0-1)"""
        import colorsys
        r, g, b = colorsys.hsv_to_rgb(h / 360.0, s, v)
        return cls(int(r * 255), int(g * 255), int(b * 255), alpha)


class ColorManager:
    """
    Manages unique color assignment for robots and their visualizations
    """
    
    # Predefined color palettes
    BRIGHT_COLORS = [
        "#FF0000",  # Red
        "#00FF00",  # Green
        "#0000FF",  # Blue
        "#FF7F00",  # Orange
        "#FF00FF",  # Magenta
        "#00FFFF",  # Cyan
        "#FFFF00",  # Yellow
        "#FF007F",  # Rose
        "#7F00FF",  # Violet
        "#00FF7F",  # Spring Green
    ]
    
    PASTEL_COLORS = [
        "#FFB3BA",  # Light Pink
        "#FFDFBA",  # Light Peach
        "#FFFFBA",  # Light Yellow
        "#BAFFC9",  # Light Green
        "#BAE1FF",  # Light Blue
        "#C9BAFF",  # Light Purple
        "#FFBAE1",  # Light Magenta
        "#E1BAFF",  # Light Lavender
        "#BAFFE1",  # Light Mint
        "#FFE1BA",  # Light Orange
    ]
    
    DARK_COLORS = [
        "#8B0000",  # Dark Red
        "#006400",  # Dark Green
        "#00008B",  # Dark Blue
        "#FF8C00",  # Dark Orange
        "#8B008B",  # Dark Magenta
        "#008B8B",  # Dark Cyan
        "#B8860B",  # Dark Goldenrod
        "#8B4513",  # Saddle Brown
        "#2F4F4F",  # Dark Slate Gray
        "#800080",  # Purple
    ]
    
    def __init__(self, color_scheme: ColorScheme = ColorScheme.BRIGHT, seed: Optional[int] = None):
        """
        Initialize color manager
        
        Args:
            color_scheme: Color scheme to use for assignments
            seed: Random seed for reproducible color generation
        """
        self.color_scheme = color_scheme
        self.robot_colors: Dict[str, RGBColor] = {}
        self.used_colors: List[str] = []
        self.color_index = 0
        
        # Set random seed if provided
        if seed is not None:
            random.seed(seed)
        
        # Select color palette based on scheme
        self.palette = self._get_palette_for_scheme(color_scheme)
    
    def _get_palette_for_scheme(self, scheme: ColorScheme) -> List[str]:
        """Get color palette for the specified scheme"""
        if scheme == ColorScheme.BRIGHT:
            return self.BRIGHT_COLORS
        elif scheme == ColorScheme.PASTEL:
            return self.PASTEL_COLORS
        elif scheme == ColorScheme.DARK:
            return self.DARK_COLORS
        elif scheme == ColorScheme.RAINBOW:
            return self._generate_rainbow_palette(12)
        elif scheme == ColorScheme.NEON:
            return self._generate_neon_palette(10)
        else:
            return self.BRIGHT_COLORS
    
    def _generate_rainbow_palette(self, count: int) -> List[str]:
        """Generate rainbow color palette"""
        colors = []
        for i in range(count):
            hue = (i * 360) / count
            color = RGBColor.from_hsv(hue, 1.0, 1.0)
            colors.append(color.to_hex())
        return colors
    
    def _generate_neon_palette(self, count: int) -> List[str]:
        """Generate neon color palette"""
        colors = []
        for i in range(count):
            hue = (i * 360) / count
            color = RGBColor.from_hsv(hue, 1.0, 1.0)
            colors.append(color.to_hex())
        return colors
    
    def get_robot_color(self, robot_name: str) -> RGBColor:
        """
        Get or assign a unique color for a robot
        
        Args:
            robot_name: Name of the robot
            
        Returns:
            RGBColor assigned to the robot
        """
        if robot_name in self.robot_colors:
            return self.robot_colors[robot_name]
        
        # Assign new color
        color = self._assign_new_color(robot_name)
        self.robot_colors[robot_name] = color
        return color
    
    def _assign_new_color(self, robot_name: str) -> RGBColor:
        """Assign a new unique color for a robot"""
        # First try predefined palette
        if self.color_index < len(self.palette):
            color_hex = self.palette[self.color_index]
            self.color_index += 1
            self.used_colors.append(color_hex)
            return RGBColor.from_hex(color_hex)
        
        # If palette exhausted, generate deterministic color based on robot name
        return self._generate_deterministic_color(robot_name)
    
    def _generate_deterministic_color(self, robot_name: str) -> RGBColor:
        """Generate a deterministic color based on robot name hash"""
        # Use robot name hash to generate consistent color
        hash_object = hashlib.md5(robot_name.encode())
        hash_hex = hash_object.hexdigest()
        
        # Extract RGB from hash
        r = int(hash_hex[0:2], 16)
        g = int(hash_hex[2:4], 16)
        b = int(hash_hex[4:6], 16)
        
        # Ensure minimum brightness
        min_brightness = 100
        if r + g + b < min_brightness * 3:
            r = max(r, min_brightness)
            g = max(g, min_brightness)
            b = max(b, min_brightness)
        
        return RGBColor(r, g, b)
    
    def get_laser_scan_color(self, robot_name: str, alpha: float = 0.8) -> RGBColor:
        """Get color for laser scan visualization with specified alpha"""
        base_color = self.get_robot_color(robot_name)
        return RGBColor(base_color.r, base_color.g, base_color.b, alpha)
    
    def get_path_color(self, robot_name: str, path_type: str = "global", alpha: float = 0.9) -> RGBColor:
        """
        Get color for path visualization
        
        Args:
            robot_name: Name of the robot
            path_type: Type of path ("global" or "local")
            alpha: Alpha transparency value
        """
        base_color = self.get_robot_color(robot_name)
        
        if path_type == "local":
            # Make local path slightly lighter/more transparent
            alpha = min(alpha * 0.7, 1.0)
            # Lighten the color for local paths
            r = min(base_color.r + 50, 255)
            g = min(base_color.g + 50, 255)
            b = min(base_color.b + 50, 255)
            return RGBColor(r, g, b, alpha)
        else:
            # Global path uses base color
            return RGBColor(base_color.r, base_color.g, base_color.b, alpha)
    
    def get_transform_color(self, robot_name: str, alpha: float = 1.0) -> RGBColor:
        """Get color for robot transform/coordinate frame visualization"""
        base_color = self.get_robot_color(robot_name)
        return RGBColor(base_color.r, base_color.g, base_color.b, alpha)
    
    def reset_robot_color(self, robot_name: str) -> None:
        """Reset color assignment for a specific robot"""
        if robot_name in self.robot_colors:
            del self.robot_colors[robot_name]
    
    def clear_all_colors(self) -> None:
        """Clear all color assignments"""
        self.robot_colors.clear()
        self.used_colors.clear()
        self.color_index = 0
    
    def get_all_robot_colors(self) -> Dict[str, RGBColor]:
        """Get all assigned robot colors"""
        return self.robot_colors.copy()
    
    def set_robot_color(self, robot_name: str, color: RGBColor) -> None:
        """Manually set color for a specific robot"""
        self.robot_colors[robot_name] = color
    
    def get_color_summary(self) -> Dict[str, str]:
        """Get summary of all color assignments"""
        return {robot: color.to_hex() for robot, color in self.robot_colors.items()}