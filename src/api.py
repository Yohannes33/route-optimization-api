from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List, Dict, Any
import assign_routes  # Import your existing code

app = FastAPI()

class LocationData(BaseModel):
    HQ: List[float]
    employees: List[List[float]]

class RouteRequest(BaseModel):
    locations: LocationData
    shuttle_capacities: List[int]

@app.post("/assign-routes")
async def assign_routes_endpoint(request: RouteRequest):
    try:
        # Prepare locations
        locations = [request.locations.HQ] + request.locations.employees
        
        # Calculate matrices
        distance_matrix, bearing_matrix = assign_routes.calculate_distance_and_bearing_matrix(locations)
        
        # Assign routes
        routes = assign_routes.assign_employees_to_shuttles(
            locations, 
            distance_matrix, 
            bearing_matrix, 
            request.shuttle_capacities
        )
        
        if not routes:
            raise HTTPException(status_code=400, detail="No solution found")
            
        # Verify assignments
        num_employees = len(request.locations.employees)
        verification_passed = assign_routes.verify_unique_assignments(routes, num_employees)
        
        return {
            "success": True,
            "routes": routes,
            "verification_passed": verification_passed,
            "total_demand": len(locations) - 1,
            "total_capacity": sum(request.shuttle_capacities)
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/")
async def root():
    return {"message": "Route Assignment API is running"}