# Shared library for camera-based object detection.
# Implemented by: Abdullateef Vahora

def is_trash_visible(camera):
    """Analyses the camera's view for a cluster of blue pixels."""
    image_data = camera.getImageArray()
    
    # Camera Check and Get Real Dimensions
    try:
        actual_height = len(image_data)
        actual_width = len(image_data[0])
        if actual_height == 0 or actual_width == 0:
            return False
    except (IndexError, TypeError):
        return False  # Camera not ready
        
    # Constants
    MIN_BRIGHTNESS = 40
    BLUE_PURITY = 15
    MIN_CLUSTER_SIZE = 5  # Minimum 5 pixels for a valid ball

    visited = set()
    
    current_cluster_count = 0

    def _is_pixel_blue(y, x):
        """Checks if a single pixel at (y, x) matches the "blue" criteria."""
        try:
            R, G, B = image_data[y][x]
            
            # Filter out black or very dark pixels
            if B < MIN_BRIGHTNESS:
                return False

            # Check if the pixel is predominantly blue
            if B > R and B > G:
                # Check it is not just grey-blue for eg.
                avg_rg = (R + G) // 2
                if B > (avg_rg + BLUE_PURITY):
                    return True  
            
            return False
        except IndexError:
            return False  # Out of bounds

    def _dfs_flood_fill(y, x):
        """
        Performs a DFS (flood-fill) to find the size of a
        connected cluster of blue pixels.
        """
        nonlocal current_cluster_count
        
        # Check bounds
        if not (0 <= y < actual_height and 0 <= x < actual_width):
            return
        
        # Check if already visited
        if (y, x) in visited:
            return
            
        visited.add((y, x))
        
        if not _is_pixel_blue(y, x):
            return
        
        # This is a valid blue pixel
        current_cluster_count += 1
        
        # 2. Explore all 4 immediate neighbors
        _dfs_flood_fill(y + 1, x) 
        _dfs_flood_fill(y - 1, x)  
        _dfs_flood_fill(y, x + 1)  
        _dfs_flood_fill(y, x - 1)  


    # Scan every 3rd pixel to find starting points efficiently
    y_quarter = actual_height // 4
    for y in range(y_quarter, y_quarter * 3, 3):
        for x in range(0, actual_width, 3):
            if (y, x) not in visited and _is_pixel_blue(y, x):
                
                current_cluster_count = 0
                
                # 2. Launch DFS to find the whole cluster
                _dfs_flood_fill(y, x) 
                
                # Check if it's large enough
                if current_cluster_count > MIN_CLUSTER_SIZE:
                    return True
                        
    # If the loops finish, we found no valid balls
    return False