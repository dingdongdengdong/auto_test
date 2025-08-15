#!/usr/bin/env python3
"""
Path Planner for Road Network
Finds optimal routes between road links using node connectivity
"""

import json
import math
from typing import List, Dict, Set, Optional, Tuple
from collections import defaultdict, deque

class RoadPathPlanner:
    """
    Path planner for road network navigation
    """
    
    def __init__(self, link_set_file: str):
        """
        Initialize path planner with road data
        
        Args:
            link_set_file: Path to link_set.json file
        """
        self.links = self._load_road_data(link_set_file)
        self.node_to_links = self._build_connectivity_map()
        self.link_to_nodes = self._build_link_node_map()
        
    def _load_road_data(self, file_path: str) -> List[Dict]:
        """Load road data from JSON file"""
        try:
            with open(file_path, 'r') as f:
                data = json.load(f)
            print(f"Loaded {len(data)} road links")
            return data
        except Exception as e:
            print(f"Error loading road data: {e}")
            return []
    
    def _build_connectivity_map(self) -> Dict[str, List[str]]:
        """Build map from nodes to connected links"""
        node_to_links = defaultdict(list)
        
        for link in self.links:
            from_node = link['from_node_idx']
            to_node = link['to_node_idx']
            link_id = link['idx']
            
            # Add links that start from this node
            node_to_links[from_node].append(link_id)
            # Add links that end at this node
            node_to_links[to_node].append(link_id)
        
        return dict(node_to_links)
    
    def _build_link_node_map(self) -> Dict[str, Dict[str, str]]:
        """Build map from link IDs to their start/end nodes"""
        link_to_nodes = {}
        
        for link in self.links:
            link_to_nodes[link['idx']] = {
                'from': link['from_node_idx'],
                'to': link['to_node_idx']
            }
        
        return link_to_nodes
    
    def find_link_by_id(self, link_id: str) -> Optional[Dict]:
        """Find link data by its ID"""
        for link in self.links:
            if link['idx'] == link_id:
                return link
        return None
    
    def find_link_by_index(self, index: int) -> Optional[Dict]:
        """Find link data by its array index"""
        if 0 <= index < len(self.links):
            return self.links[index]
        return None
    
    def get_connected_links(self, link_id: str) -> List[str]:
        """
        Get all links that can be reached from a given link
        
        Args:
            link_id: ID of the source link
            
        Returns:
            List of link IDs that can be reached
        """
        if link_id not in self.link_to_nodes:
            return []
        
        end_node = self.link_to_nodes[link_id]['to']
        connected_links = []
        
        # Find all links that start from the end node of current link
        for link in self.links:
            if link['from_node_idx'] == end_node:
                connected_links.append(link['idx'])
        
        return connected_links
    
    def find_path_between_links(self, start_link_id: str, end_link_id: str, 
                               max_depth: int = 10) -> Optional[List[str]]:
        """
        Find path between two road links using BFS
        
        Args:
            start_link_id: Starting link ID
            end_link_id: Target link ID
            max_depth: Maximum search depth
            
        Returns:
            List of link IDs forming the path, or None if no path found
        """
        if start_link_id == end_link_id:
            return [start_link_id]
        
        # BFS to find shortest path
        queue = deque([(start_link_id, [start_link_id])])
        visited = set()
        
        while queue and len(queue[0][1]) <= max_depth:
            current_link, path = queue.popleft()
            
            if current_link in visited:
                continue
            
            visited.add(current_link)
            
            # Get connected links
            connected_links = self.get_connected_links(current_link)
            
            for next_link in connected_links:
                if next_link == end_link_id:
                    return path + [next_link]
                
                if next_link not in visited:
                    queue.append((next_link, path + [next_link]))
        
        return None
    
    def find_path_via_waypoints(self, waypoint_links: List[str]) -> Optional[List[str]]:
        """
        Find path that goes through specific waypoint links in order
        
        Args:
            waypoint_links: List of link IDs to visit in order
            
        Returns:
            Complete path connecting all waypoints, or None if impossible
        """
        if len(waypoint_links) < 2:
            return waypoint_links
        
        complete_path = []
        
        for i in range(len(waypoint_links) - 1):
            start_link = waypoint_links[i]
            end_link = waypoint_links[i + 1]
            
            # Find path between these two waypoints
            segment_path = self.find_path_between_links(start_link, end_link)
            
            if segment_path is None:
                print(f"❌ No path found between {start_link} and {end_link}")
                return None
            
            # Add segment to complete path (avoid duplicate waypoints)
            if i == 0:
                complete_path.extend(segment_path)
            else:
                complete_path.extend(segment_path[1:])  # Skip first to avoid duplicate
        
        return complete_path
    
    def analyze_route(self, route_links: List[str]) -> Dict:
        """
        Analyze a route for distance, time, and details
        
        Args:
            route_links: List of link IDs forming the route
            
        Returns:
            Dictionary with route analysis
        """
        total_distance = 0
        total_time = 0
        route_details = []
        
        for link_id in route_links:
            link_data = self.find_link_by_id(link_id)
            if link_data:
                distance = link_data.get('link_length', 0)
                max_speed = float(link_data.get('max_speed', '20'))
                
                # Convert speed to m/s if needed
                if max_speed > 50:  # Likely km/h
                    max_speed = max_speed / 3.6
                
                time = distance / max_speed if max_speed > 0 else 0
                
                total_distance += distance
                total_time += time
                
                route_details.append({
                    'link_id': link_id,
                    'distance': distance,
                    'max_speed': max_speed,
                    'time': time,
                    'from_node': link_data['from_node_idx'],
                    'to_node': link_data['to_node_idx']
                })
        
        return {
            'total_distance': total_distance,
            'total_time': total_time,
            'link_count': len(route_links),
            'route_details': route_details
        }
    
    def print_route_analysis(self, route_links: List[str], title: str = "Route Analysis"):
        """Print detailed analysis of a route"""
        print(f"\n{'='*50}")
        print(f"  {title}")
        print(f"{'='*50}")
        
        if not route_links:
            print("❌ No route found!")
            return
        
        analysis = self.analyze_route(route_links)
        
        print(f"📏 총 거리: {analysis['total_distance']:.2f} m")
        print(f"⏱️  예상 시간: {analysis['total_time']:.2f} s")
        print(f"🛣️  경유 링크 수: {analysis['link_count']}")
        
        print(f"\n📋 상세 경로:")
        for i, detail in enumerate(analysis['route_details']):
            print(f"  {i+1:2d}. {detail['link_id']}")
            print(f"      거리: {detail['distance']:.2f}m, 속도: {detail['max_speed']:.1f}m/s")
            print(f"      {detail['from_node']} → {detail['to_node']}")
            print()

def main():
    """Example usage of the path planner"""
    
    print("🛣️  Road Network Path Planner")
    print("="*50)
    
    # Initialize planner
    planner = RoadPathPlanner("link_set.json")
    
    if not planner.links:
        print("❌ Failed to load road data!")
        return
    
    # Test 1: Find path from 1번 링크 to 23번 링크
    print("\n🔍 테스트 1: 1번 링크 → 23번 링크 경로 찾기")
    
    link_1 = planner.find_link_by_index(0)  # 1번 링크
    link_23 = planner.find_link_by_index(21)  # 23번 링크
    
    if link_1 and link_23:
        print(f"1번 링크: {link_1['idx']} ({link_1['from_node_idx']} → {link_1['to_node_idx']})")
        print(f"23번 링크: {link_23['idx']} ({link_23['from_node_idx']} → {link_23['to_node_idx']})")
        
        path_1_to_23 = planner.find_path_between_links(link_1['idx'], link_23['idx'])
        if path_1_to_23:
            print(f"✅ 경로 발견: {' → '.join(path_1_to_23)}")
            planner.print_route_analysis(path_1_to_23, "1번 → 23번 링크 경로")
        else:
            print("❌ 경로를 찾을 수 없습니다")
    
    # Test 2: Find path from 23번 링크 to 13번 링크
    print("\n🔍 테스트 2: 23번 링크 → 13번 링크 경로 찾기")
    
    link_13 = planner.find_link_by_index(12)  # 13번 링크
    
    if link_23 and link_13:
        print(f"23번 링크: {link_23['idx']} ({link_23['from_node_idx']} → {link_23['to_node_idx']})")
        print(f"13번 링크: {link_13['idx']} ({link_13['from_node_idx']} → {link_13['to_node_idx']})")
        
        path_23_to_13 = planner.find_path_between_links(link_23['idx'], link_13['idx'])
        if path_23_to_13:
            print(f"✅ 경로 발견: {' → '.join(path_23_to_13)}")
            planner.print_route_analysis(path_23_to_13, "23번 → 13번 링크 경로")
        else:
            print("❌ 경로를 찾을 수 없습니다")
    
    # Test 3: Find complete path 1번 → 23번 → 13번
    print("\n🔍 테스트 3: 1번 → 23번 → 13번 링크 전체 경로 찾기")
    
    if link_1 and link_23 and link_13:
        waypoints = [link_1['idx'], link_23['idx'], link_13['idx']]
        complete_path = planner.find_path_via_waypoints(waypoints)
        
        if complete_path:
            print(f"✅ 전체 경로 발견: {' → '.join(complete_path)}")
            planner.print_route_analysis(complete_path, "1번 → 23번 → 13번 링크 전체 경로")
        else:
            print("❌ 전체 경로를 찾을 수 없습니다")
    
    # Test 4: Show connectivity for key nodes
    print("\n🔍 테스트 4: 주요 노드 연결성 분석")
    
    if link_1 and link_23:
        print(f"\n1번 링크 끝 노드 ({link_1['to_node_idx']})에서 연결 가능한 링크들:")
        connected_from_1 = planner.get_connected_links(link_1['idx'])
        for link_id in connected_from_1:
            link_data = planner.find_link_by_id(link_id)
            if link_data:
                print(f"  → {link_id}: {link_data['from_node_idx']} → {link_data['to_node_idx']}")
        
        print(f"\n23번 링크 시작 노드 ({link_23['from_node_idx']})에서 연결 가능한 링크들:")
        # Find links that start from this node
        for link in planner.links:
            if link['from_node_idx'] == link_23['from_node_idx']:
                print(f"  → {link['idx']}: {link['from_node_idx']} → {link['to_node_idx']}")

if __name__ == "__main__":
    main()
