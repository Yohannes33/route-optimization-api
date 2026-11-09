# Shuttle Route Optimization System

An intelligent routing system that optimizes shuttle routes for employee pickup and drop-off services. The system uses Google OR-Tools to calculate efficient routes while respecting vehicle capacities and geographic distances.
## Features
- 🚐 Optimizes routes for multiple shuttles simultaneously
- 📍 Handles real-world geographic coordinates
- ⚖️ Respects vehicle capacity constraints
- 🧭 Computes bearings between stops (groundwork for direction-aware routing; not yet weighted in the route cost)
- 🔄 Ensures all employees are assigned exactly once
- 🌐 RESTful API interface
- ✅ Built-in route verification
 
## Technology Stack
- Python 3.x
- Google OR-Tools for route optimization
- FastAPI for REST API
- Haversine formula for geographic calculations
- JSON for data storage and API communication


