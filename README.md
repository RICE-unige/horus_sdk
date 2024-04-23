# Horus SDK

The Horus Software Development Kit (SDK) provides tools and libraries for developers to connect of the Horus Mixed Reality Application and to create custom applications that interact with Horus components.

**Key Components:**

* **Robot Communication API:** Functions for sending commands and retrieving state information from robots.
* **Data Structures:**  Representations for robot data, sensor data, commands, etc.
* **Utilities:** Helper functions for common tasks related to robot interaction (e.g.,  coordinate transformations, sensor data processing, logging).

**Languages _(to be)_ Supported:**
* Python
* C++

**Getting Started:**

1. Follow the setup instructions in the main `horus_desktop` repository to ensure dependencies are installed.

2. Install the Horus SDK package:

   * **Python:** `pip install horus_sdk`
   * **C++:** 

3. Import/include the relevant Horus SDK components in your application:

   ```python
   import horus as hr
   ```
   ```cpp
   #include <horus/robot_state.hpp>
    ```