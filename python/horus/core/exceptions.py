"""SDK-specific exception types for HORUS."""


class HorusError(Exception):
    """Base class for HORUS SDK errors."""


class HorusConfigurationError(HorusError):
    """Raised when an SDK or robot configuration is invalid."""


class HorusConnectionError(HorusError):
    """Raised when HORUS cannot connect to a required service."""


class HorusRegistrationError(HorusError):
    """Raised when robot registration fails or times out."""


class HorusBridgeError(HorusConnectionError):
    """Raised when the HORUS ROS bridge cannot be started or reached."""
