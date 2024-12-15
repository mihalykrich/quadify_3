# src/managers/tidal_manager.py

from managers.menus.base_manager import BaseManager
import logging
from PIL import ImageFont
import threading
import time

class TidalManager(BaseManager):
    def __init__(self, display_manager, volumio_listener, mode_manager, window_size=4, y_offset=5, line_spacing=15):
        super().__init__(display_manager, volumio_listener, mode_manager)
        self.mode_name = "tidal"
        self.tidal_playlists = []
        self.current_selection_index = 0
        self.font_key = 'menu_font'  # Define in config.yaml under fonts
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.setLevel(logging.INFO)
        self.logger.info("TidalManager initialized.")

        # Initialize state
        self.menu_stack = []
        self.current_menu_items = []
        self.is_active = False

        # Window settings
        self.window_size = window_size
        self.window_start_index = 0

        # Display settings
        self.y_offset = y_offset
        self.line_spacing = line_spacing

        # Font settings
        self.font_key = 'menu_font'

        # Register mode change callback
        if hasattr(self.mode_manager, "add_on_mode_change_callback"):
            self.mode_manager.add_on_mode_change_callback(self.handle_mode_change)

    def start_mode(self):
        """Start Tidal mode by fetching the root navigation."""
        if self.is_active:
            self.logger.debug("TidalManager: Tidal mode already active.")
            return

        self.logger.info("TidalManager: Starting Tidal mode.")
        self.is_active = True
        self.current_selection_index = 0
        self.window_start_index = 0

        # Connect signals
        self.volumio_listener.tidal_navigation_received.connect(self.update_tidal_menu)
        self.volumio_listener.toast_message_received.connect(self.handle_toast_message)
        self.logger.debug("TidalManager: Connected to 'tidal_navigation_received' and 'toast_message_received' signals.")
        self.volumio_listener.navigation_received.connect(self.handle_navigation)

        self.volumio_listener.state_changed.connect(self.handle_state_change)
        self.volumio_listener.track_changed.connect(self.handle_track_change)

        self.display_loading_screen()
        self.fetch_tidal_navigation()  # Fetch Tidal root navigation

    def stop_mode(self):
        """Stop Tidal mode and clear the display."""
        if not self.is_active:
            self.logger.debug("TidalManager: Tidal mode already inactive.")
            return
        self.is_active = False
        self.display_manager.clear_screen()
        self.logger.info("TidalManager: Stopped Tidal mode and cleared display.")

        # Disconnect signals
        self.volumio_listener.tidal_navigation_received.disconnect(self.update_tidal_menu)
        self.volumio_listener.toast_message_received.disconnect(self.handle_toast_message)
        self.logger.debug("TidalManager: Disconnected from 'tidal_navigation_received' and 'toast_message_received' signals.")

        self.volumio_listener.state_changed.disconnect(self.handle_state_change)
        self.volumio_listener.track_changed.disconnect(self.handle_track_change)


    def handle_state_change(self, sender, state, **kwargs):
        if state.get('service') == 'tidal':
            self.logger.info("TidalManager: State changed, updating display.")
            self.update_song_info(state)
    
    def handle_track_change(self, sender, track, **kwargs):
        """Handle track changes from Volumio."""
        if track.get('service') == 'qobuz':
            self.logger.info("QobuzManager: Track changed, updating display.")
            self.update_song_info(track)

    def handle_navigation(self, sender, navigation, service, uri, **kwargs):
            if service != 'tidal':
                return  # Ignore navigation data not related to Tidal
            self.logger.info("TidalManager: Received navigation data.")
            self.update_tidal_menu(sender, navigation)

    def fetch_tidal_navigation(self, uri="tidal://"):
        """Fetch navigation data for the given URI."""
        self.logger.info(f"TidalManager: Fetching navigation data for URI: {uri}")
        if self.volumio_listener.is_connected():
            try:
                self.volumio_listener.fetch_browse_library(uri)
                self.logger.debug(f"TidalManager: Emitted 'browseLibrary' for URI: {uri}")
            except Exception as e:
                self.logger.error(f"TidalManager: Failed to emit 'browseLibrary' for {uri}: {e}")
                self.display_error_message("Navigation Error", f"Could not fetch navigation: {e}")
        else:
            self.logger.warning("TidalManager: Cannot fetch navigation - not connected to Volumio.")
            self.display_error_message("Connection Error", "Not connected to Volumio.")

    def get_visible_window(self, items):
        """Returns a subset of items to display based on the current selection."""
        if self.current_selection_index < self.window_start_index:
            self.window_start_index = self.current_selection_index
        elif self.current_selection_index >= self.window_start_index + self.window_size:
            self.window_start_index = self.current_selection_index - self.window_size + 1

        self.window_start_index = max(0, self.window_start_index)
        self.window_start_index = min(self.window_start_index, max(0, len(items) - self.window_size))

        visible_items = items[self.window_start_index:self.window_start_index + self.window_size]
        self.logger.debug(f"TidalManager: Visible window indices {self.window_start_index} to {self.window_start_index + self.window_size -1}")
        return visible_items

    def handle_mode_change(self, current_mode):
        """Handle mode changes between Tidal and other modes."""
        self.logger.info(f"TidalManager handling mode change to: {current_mode}")
        if current_mode == "tidal":
            self.logger.info("TidalManager: Entering Tidal mode...")
            self.start_mode()
        elif self.is_active:
            self.logger.info("TidalManager: Exiting Tidal mode...")
            self.stop_mode()

    def display_loading_screen(self):
        """Display a loading screen."""
        self.logger.info("TidalManager: Displaying loading screen.")

        def draw(draw_obj):
            draw_obj.text(
                (10, self.y_offset),
                "Loading Tidal...",
                font=self.display_manager.fonts.get(self.font_key, ImageFont.load_default()),
                fill="white"
            )

        self.display_manager.draw_custom(draw)

    def update_tidal_menu(self, sender, navigation, **kwargs):
        """Handle the 'tidal_navigation_received' signal and update the Tidal menu."""
        if not navigation:
            self.logger.warning("TidalManager: No navigation data received.")
            self.display_no_items()
            return

        lists = navigation.get("lists", [])

        if not lists or not isinstance(lists, list):
            self.logger.warning("TidalManager: Navigation data has no valid lists.")
            self.display_no_items()
            return

        # Combine items from all lists
        combined_items = []
        for lst in lists:
            list_items = lst.get("items", [])
            if list_items:
                combined_items.extend(list_items)

        self.logger.debug("TidalManager: Displaying combined menu items from all lists.")

        if not combined_items:
            self.logger.info("TidalManager: No items in navigation. Displaying 'No Results'.")
            self.display_no_items()
            return

        self.current_menu_items = [
            {
                "title": item.get("title", "Untitled"),
                "uri": item.get("uri", ""),
                "type": item.get("type", "")
            }
            for item in combined_items
        ]

        self.logger.info(f"TidalManager: Updated menu with {len(self.current_menu_items)} items.")

        if self.is_active:
            self.display_menu()

    def display_no_items(self):
        """Display a message if no items are available."""
        self.logger.info("TidalManager: No items available.")

        def draw(draw_obj):
            draw_obj.text(
                (10, self.y_offset),
                "No Tidal Items Available",
                font=self.display_manager.fonts.get(self.font_key, ImageFont.load_default()),
                fill="white"
            )

        self.display_manager.draw_custom(draw)

    def display_menu(self):
        """Display the current menu."""
        self.logger.info("TidalManager: Displaying menu.")
        visible_items = self.get_visible_window(self.current_menu_items)

        def draw(draw_obj):
            for i, item in enumerate(visible_items):
                actual_index = self.window_start_index + i
                arrow = "-> " if actual_index == self.current_selection_index else "   "
                title = item['title']
                self.logger.debug(f"TidalManager: Drawing item {i}: {title}")
                draw_obj.text(
                    (10, self.y_offset + i * self.line_spacing),
                    f"{arrow}{title}",
                    font=self.display_manager.fonts.get(self.font_key, ImageFont.load_default()),
                    fill="white" if actual_index == self.current_selection_index else "gray"
                )

        self.display_manager.draw_custom(draw)

    def scroll_selection(self, direction):
        """Scroll through the menu items."""
        if not self.is_active:
            self.logger.debug("TidalManager: Scroll attempted while not active.")
            return

        previous_index = self.current_selection_index
        self.current_selection_index += direction
        self.current_selection_index = max(0, min(self.current_selection_index, len(self.current_menu_items) - 1))
        self.logger.debug(f"TidalManager: Scrolled from {previous_index} to {self.current_selection_index}.")
        self.display_menu()

    def select_item(self):
        """Handle the selection of the current menu item."""
        if not self.is_active or not self.current_menu_items:
            self.logger.warning("TidalManager: Select attempted with no items.")
            return

        selected_item = self.current_menu_items[self.current_selection_index]
        self.logger.info(f"TidalManager: Selected item: {selected_item}")
        uri = selected_item.get("uri")

        if not uri:
            self.logger.warning("TidalManager: Selected item has no URI.")
            self.display_error_message("Invalid Selection", "Selected item has no URI.")
            return

        # Determine if the item is a song by checking the URI prefix
        if uri.startswith("tidal://song/"):
            self.logger.info(f"TidalManager: Playing song with URI: {uri}")
            self.play_song(uri)
            return

        # Alternatively, check if the type indicates it's a song
        if selected_item.get("type", "").lower() == "song":
            self.logger.info(f"TidalManager: Playing song with URI: {uri}")
            self.play_song(uri)
            return

        # If not a song, treat it as a category or folder and navigate into it
        self.logger.info(f"TidalManager: Navigating into category with URI: {uri}")
        self.navigate_to(uri)

    def play_song(self, uri):
        """Emit commands to Volumio to play the selected song directly."""
        self.logger.info(f"TidalManager: Sending play commands for URI: {uri}")
        if self.volumio_listener.is_connected():
            try:
                # Suppress state changes
                self.mode_manager.suppress_state_change()

                # Step 1: Clear the current queue
                self.volumio_listener.socketIO.emit('clearQueue')
                self.logger.info("TidalManager: Cleared Volumio queue.")

                # Step 2: Add the selected track to the queue
                selected_item = self.current_menu_items[self.current_selection_index]
                song_title = selected_item.get("title", "Untitled")
                self.volumio_listener.socketIO.emit('addToQueue', {
                    "service": "tidal",
                    "uri": uri,
                    "title": song_title
                })
                self.logger.info(f"TidalManager: Added '{song_title}' to Volumio queue.")

                # Step 3: Start playback
                self.volumio_listener.socketIO.emit('play')
                self.logger.info("TidalManager: Sent play command to Volumio.")

                # Allow state changes after a short delay
                threading.Timer(1.0, self.mode_manager.allow_state_change).start()
            except Exception as e:
                self.logger.error(f"TidalManager: Failed to play track {uri}: {e}")
                self.display_error_message("Playback Error", f"Could not play track: {e}")
        else:
            self.logger.warning("TidalManager: Cannot play song - not connected to Volumio.")
            self.display_error_message("Connection Error", "Not connected to Volumio.")

    def navigate_to(self, uri):
        """Navigate into a submenu or playlist."""
        # Push current menu context to stack for back functionality
        self.menu_stack.append({
            "menu_items": self.current_menu_items.copy(),
            "selection_index": self.current_selection_index,
            "window_start_index": self.window_start_index
        })
        self.logger.debug("TidalManager: Pushed current menu context to stack.")

        # Fetch navigation data for the new URI
        self.logger.info(f"TidalManager: Fetching navigation data for URI: {uri}")
        self.display_loading_screen()
        self.fetch_tidal_navigation(uri)

    def go_back(self):
        """Navigate back to the previous menu context."""
        if not self.menu_stack:
            self.logger.info("TidalManager: No previous menu to return to.")
            self.stop_mode()
            return

        # Restore previous menu context
        previous_context = self.menu_stack.pop()
        self.current_menu_items = previous_context["menu_items"]
        self.current_selection_index = previous_context["selection_index"]
        self.window_start_index = previous_context["window_start_index"]
        self.logger.debug("TidalManager: Restored previous menu context from stack.")
        self.display_menu()

    def handle_toast_message(self, sender, message, **kwargs):
        """Handle toast messages from Volumio, especially errors."""
        message_type = message.get("type", "")
        title = message.get("title", "")
        body = message.get("message", "")

        if message_type == "error":
            self.logger.error(f"TidalManager: Error received - {title}: {body}")
            self.display_error_message(title, body)
        elif message_type == "success":
            self.logger.info(f"TidalManager: Success - {title}: {body}")
            # Optionally handle success messages (e.g., confirmations)
        else:
            self.logger.info(f"TidalManager: Received toast message - {title}: {body}")

    def display_error_message(self, title, message):
        """Display an error message on the screen."""
        self.logger.info(f"TidalManager: Displaying error message: {title} - {message}")

        def draw(draw_obj):
            # Display title
            draw_obj.text(
                (10, self.y_offset),
                f"Error: {title}",
                font=self.display_manager.fonts.get(self.font_key, ImageFont.load_default()),
                fill="red"
            )
            # Display message
            draw_obj.text(
                (10, self.y_offset + self.line_spacing),
                message,
                font=self.display_manager.fonts.get(self.font_key, ImageFont.load_default()),
                fill="white"
            )

        self.display_manager.draw_custom(draw)

    def update_song_info(self, state):
        """Update the playback metrics display based on the current state."""
        self.logger.info("PlaybackManager: Updating playback metrics display.")

        # Extract relevant playback information
        sample_rate = state.get("samplerate", "Unknown Sample Rate")
        bitdepth = state.get("bitdepth", "Unknown Bit Depth")
        volume = state.get("volume", "Unknown Volume")

        # Forward the information to the PlaybackManager to handle without drawing directly here
        self.mode_manager.playback_manager.update_playback_metrics(state)