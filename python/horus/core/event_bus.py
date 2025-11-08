"""
Event bus system for decoupled component communication in HORUS SDK

This module provides a thread-safe, asynchronous event publishing and subscription system
that enables loose coupling between different SDK components, facilitating real-time
communication between ROS robots and the Meta Quest 3 HORUS application.
"""

import asyncio
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Union
from uuid import uuid4


class EventPriority(Enum):
    """Event priority levels for processing order"""
    LOW = 0
    NORMAL = 1
    HIGH = 2
    CRITICAL = 3


@dataclass
class Event:
    """
    Base event class for EventBus communication
    
    Args:
        topic: Event topic/channel identifier (e.g., "robot1.sensor.camera")
        data: Event payload data
        priority: Event processing priority
        source: Optional source identifier
        event_id: Unique event identifier
        timestamp: Event creation timestamp
    """
    topic: str
    data: Any = None
    priority: EventPriority = EventPriority.NORMAL
    source: Optional[str] = None
    event_id: str = field(default_factory=lambda: str(uuid4()))
    timestamp: float = field(default_factory=time.time)

    def __post_init__(self):
        """Validate event configuration"""
        if not self.topic:
            raise ValueError("Event topic cannot be empty")
        if not isinstance(self.priority, EventPriority):
            raise TypeError("Priority must be an EventPriority enum")


class EventSubscription:
    """
    Manages a single event subscription
    
    Args:
        callback: Function to call when event is received
        topic_filter: Topic pattern to match against
        priority_filter: Optional minimum priority level
        subscription_id: Unique subscription identifier
    """
    
    def __init__(
        self,
        callback: Callable[[Event], None],
        topic_filter: str,
        priority_filter: Optional[EventPriority] = None,
        subscription_id: Optional[str] = None
    ):
        self.callback = callback
        self.topic_filter = topic_filter
        self.priority_filter = priority_filter
        self.subscription_id = subscription_id or str(uuid4())
        self.active = True
        
    def matches(self, event: Event) -> bool:
        """Check if this subscription should receive the event"""
        if not self.active:
            return False
            
        # Check topic pattern matching
        if not self._matches_topic(event.topic, self.topic_filter):
            return False
            
        # Check priority filter
        if self.priority_filter and event.priority.value < self.priority_filter.value:
            return False
            
        return True
    
    def _matches_topic(self, event_topic: str, topic_filter: str) -> bool:
        """
        Check if event topic matches subscription filter
        
        Supports HORUS-specific patterns:
        - "*" matches everything
        - "robot.*" matches "robot.status", "robot.sensor.camera", etc.
        - "robot.+" matches "robot.status" but not "robot.sensor.camera"
        - "*.sensor.*" matches any robot's sensor data
        """
        if topic_filter == "*":
            return True
            
        # Exact match
        if event_topic == topic_filter:
            return True
            
        # Simple prefix wildcard matching
        if topic_filter.endswith("*"):
            prefix = topic_filter[:-1]  # Remove the *
            return event_topic.startswith(prefix)
            
        # Convert patterns to parts
        filter_parts = topic_filter.split(".")
        event_parts = event_topic.split(".")
        
        return self._match_parts(event_parts, filter_parts)
    
    def _match_parts(self, event_parts: List[str], filter_parts: List[str]) -> bool:
        """Match topic parts with wildcard support"""
        # Simple approach: if filter ends with *, match prefix
        if len(filter_parts) > 0 and filter_parts[-1] == "*":
            # Remove the * and check if event starts with filter prefix
            prefix_parts = filter_parts[:-1]
            if len(event_parts) >= len(prefix_parts):
                return event_parts[:len(prefix_parts)] == prefix_parts
            return False
        
        # Exact match with + wildcards
        if len(event_parts) != len(filter_parts):
            return False
            
        for event_part, filter_part in zip(event_parts, filter_parts):
            if filter_part != "+" and filter_part != event_part:
                return False
                
        return True


class EventBus:
    """
    Thread-safe event bus for HORUS SDK component communication
    
    Enables real-time data flow between ROS robots and Meta Quest 3 application
    through topic-based publish/subscribe messaging with priority handling.
    """
    
    def __init__(self, max_queue_size: int = 1000):
        self._subscriptions: Dict[str, List[EventSubscription]] = {}
        self._lock = threading.RLock()
        self._stats = {
            "events_published": 0,
            "events_processed": 0,
            "active_subscriptions": 0,
            "dropped_events": 0
        }
        self._running = False
        self._processing_thread: Optional[threading.Thread] = None
        self._event_queue: List[Event] = []
        self._queue_condition = threading.Condition(self._lock)
        self._max_queue_size = max_queue_size
        
    def start(self):
        """Start the event processing system"""
        if self._running:
            return
            
        self._running = True
        self._processing_thread = threading.Thread(target=self._process_events, daemon=True)
        self._processing_thread.start()
        
    def stop(self):
        """Stop the event processing system"""
        self._running = False
        with self._queue_condition:
            self._queue_condition.notify_all()
            
        if self._processing_thread:
            self._processing_thread.join(timeout=1.0)
            
    def _process_events(self):
        """Main event processing loop"""
        while self._running:
            event = None
            
            with self._queue_condition:
                # Wait for events or timeout
                if not self._event_queue and self._running:
                    self._queue_condition.wait(timeout=0.1)
                
                if self._event_queue and self._running:
                    # Sort by priority (higher priority first)
                    self._event_queue.sort(key=lambda e: e.priority.value, reverse=True)
                    event = self._event_queue.pop(0)
            
            if event:
                self._dispatch_event(event)
                self._stats["events_processed"] += 1
                
    def _dispatch_event(self, event: Event):
        """Dispatch event to matching subscriptions"""
        matching_subscriptions = []
        
        with self._lock:
            # Find all matching subscriptions
            for subscriptions in self._subscriptions.values():
                for subscription in subscriptions[:]:  # Copy to avoid modification during iteration
                    if subscription.matches(event):
                        matching_subscriptions.append(subscription)
        
        # Call callbacks (outside of lock to avoid deadlocks)
        for subscription in matching_subscriptions:
            try:
                subscription.callback(event)
            except Exception as e:
                print(f"EventBus callback error for {event.topic}: {e}")
                
    def publish(self, event: Union[Event, str], data: Any = None, priority: EventPriority = EventPriority.NORMAL, source: Optional[str] = None):
        """
        Publish an event to the bus
        
        Args:
            event: Event object or topic string (e.g., "robot1.sensor.camera")
            data: Event data (sensor reading, status update, etc.)
            priority: Event priority level
            source: Optional event source identifier
            
        Examples:
            # Robot sensor data
            publish("robot1.sensor.camera", camera_image_data)
            
            # Robot status update
            publish("robot1.status.battery", {"level": 85, "charging": False})
            
            # Quest 3 command
            publish("command.robot1.move", {"x": 1.0, "y": 0.0}, priority=EventPriority.HIGH)
        """
        if isinstance(event, str):
            event = Event(topic=event, data=data, priority=priority, source=source)
        elif not isinstance(event, Event):
            raise TypeError("Event must be Event object or topic string")
            
        with self._queue_condition:
            if len(self._event_queue) >= self._max_queue_size:
                # Drop oldest low-priority event to make room
                for i, queued_event in enumerate(self._event_queue):
                    if queued_event.priority == EventPriority.LOW:
                        self._event_queue.pop(i)
                        self._stats["dropped_events"] += 1
                        break
                else:
                    # If no low priority events, drop oldest
                    if self._event_queue:
                        self._event_queue.pop(0)
                        self._stats["dropped_events"] += 1
            
            self._event_queue.append(event)
            self._stats["events_published"] += 1
            self._queue_condition.notify()
            
    def subscribe(
        self,
        topic_filter: str,
        callback: Callable[[Event], None],
        priority_filter: Optional[EventPriority] = None
    ) -> str:
        """
        Subscribe to events matching topic filter
        
        Args:
            topic_filter: Topic pattern to match (supports * and + wildcards)
            callback: Function to call when matching event is received
            priority_filter: Optional minimum priority level
            
        Returns:
            Subscription ID for later unsubscription
            
        Examples:
            # Subscribe to all robot sensor data
            subscribe("*.sensor.*", handle_sensor_data)
            
            # Subscribe to specific robot status
            subscribe("robot1.status.*", handle_robot1_status)
            
            # Subscribe to all high-priority events
            subscribe("*", handle_critical_events, priority_filter=EventPriority.HIGH)
        """
        subscription = EventSubscription(callback, topic_filter, priority_filter)
        
        with self._lock:
            if topic_filter not in self._subscriptions:
                self._subscriptions[topic_filter] = []
            self._subscriptions[topic_filter].append(subscription)
            self._stats["active_subscriptions"] = sum(len(subs) for subs in self._subscriptions.values())
            
        try:
            # Update live topic board (TTY-aware, no-op if unavailable)
            from horus.utils.topic_status import get_topic_status_board

            get_topic_status_board().on_subscribe(topic_filter)
        except Exception:
            pass
        return subscription.subscription_id
        
    def unsubscribe(self, subscription_id: str) -> bool:
        """
        Unsubscribe from events
        
        Args:
            subscription_id: ID returned from subscribe()
            
        Returns:
            True if subscription was found and removed
        """
        with self._lock:
            for topic, subscriptions in self._subscriptions.items():
                for i, subscription in enumerate(subscriptions):
                    if subscription.subscription_id == subscription_id:
                        subscriptions.pop(i)
                        if not subscriptions:
                            del self._subscriptions[topic]
                        self._stats["active_subscriptions"] = sum(len(subs) for subs in self._subscriptions.values())
                        try:
                            from horus.utils.topic_status import get_topic_status_board

                            get_topic_status_board().on_unsubscribe(topic)
                        except Exception:
                            pass
                        return True
        return False
        
    def unsubscribe_all(self, topic_filter: Optional[str] = None):
        """
        Unsubscribe all subscriptions, optionally filtered by topic
        
        Args:
            topic_filter: If provided, only unsubscribe from this topic
        """
        with self._lock:
            if topic_filter:
                if topic_filter in self._subscriptions:
                    del self._subscriptions[topic_filter]
                    try:
                        from horus.utils.topic_status import get_topic_status_board

                        get_topic_status_board().on_unsubscribe(topic_filter)
                    except Exception:
                        pass
            else:
                # Notify for each topic key being removed to keep board consistent
                try:
                    from horus.utils.topic_status import get_topic_status_board

                    for t in list(self._subscriptions.keys()):
                        get_topic_status_board().on_unsubscribe(t)
                except Exception:
                    pass
                self._subscriptions.clear()
            self._stats["active_subscriptions"] = sum(len(subs) for subs in self._subscriptions.values())
            
    def get_stats(self) -> Dict[str, Any]:
        """Get event bus statistics"""
        with self._lock:
            stats = self._stats.copy()
            stats["queue_size"] = len(self._event_queue)
            return stats
        
    def get_subscriptions(self) -> Dict[str, int]:
        """Get current subscription counts by topic"""
        with self._lock:
            return {topic: len(subs) for topic, subs in self._subscriptions.items()}


# Global event bus instance for HORUS SDK
_global_event_bus: Optional[EventBus] = None


def get_event_bus() -> EventBus:
    """Get the global HORUS event bus instance"""
    global _global_event_bus
    if _global_event_bus is None:
        _global_event_bus = EventBus()
        _global_event_bus.start()
    return _global_event_bus


def publish(topic: str, data: Any = None, priority: EventPriority = EventPriority.NORMAL, source: Optional[str] = None):
    """
    Convenience function to publish to global event bus
    
    Examples:
        # Robot publishes sensor data
        publish("robot1.sensor.lidar", point_cloud_data)
        
        # Quest 3 sends robot command
        publish("command.robot2.goto", {"x": 5.0, "y": 3.0}, priority=EventPriority.HIGH)
    """
    get_event_bus().publish(topic, data, priority, source)


def subscribe(topic_filter: str, callback: Callable[[Event], None], priority_filter: Optional[EventPriority] = None) -> str:
    """
    Convenience function to subscribe to global event bus
    
    Examples:
        # Quest 3 subscribes to all robot status updates
        subscribe("*.status.*", update_quest_ui)
        
        # DataViz subscribes to sensor data
        subscribe("*.sensor.*", update_visualization)
    """
    return get_event_bus().subscribe(topic_filter, callback, priority_filter)


def unsubscribe(subscription_id: str) -> bool:
    """Convenience function to unsubscribe from global event bus"""
    return get_event_bus().unsubscribe(subscription_id)
