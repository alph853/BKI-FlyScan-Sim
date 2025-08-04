import os
import subprocess
import yaml
import datetime
import sys
import argparse
from dataclasses import dataclass, field, asdict
from typing import List, Optional, Dict
from rich.progress import track, Progress, TaskID
from rich.console import Console

@dataclass
class TopicInfo:
    name: str
    type: str = ""
    publishers: List[str] = field(default_factory=list)
    subscribers: List[str] = field(default_factory=list)
    publisher_count: int = 0
    subscriber_count: int = 0

@dataclass
class ServiceInfo:
    name: str
    type: str = ""
    request_type: str = ""
    response_type: str = ""

@dataclass
class GazeboTopicInfo:
    name: str
    type: str = ""
    publishers: int = 0
    subscribers: int = 0

@dataclass
class ROS2Data:
    topics: List[TopicInfo] = field(default_factory=list)
    services: List[ServiceInfo] = field(default_factory=list)
    nodes: List[str] = field(default_factory=list)

@dataclass
class GazeboData:
    topics: List[GazeboTopicInfo] = field(default_factory=list)
    services: List[ServiceInfo] = field(default_factory=list)

def run_command(command: List[str]) -> Optional[str]:
    try:
        result = subprocess.run(command, capture_output=True, text=True, check=True)
        return result.stdout.strip() or None
    except (subprocess.CalledProcessError, FileNotFoundError) as e:
        Console().print(f"[red]Error running command {' '.join(command)}: {e}[/red]")
        return None

def safe_lines(output: Optional[str]) -> List[str]:
    return [line.strip() for line in (output or "").split('\n') if line.strip()]

class TopicTracker:
    def __init__(self, output_file: str = "topic_tracking.yaml"):
        self.output_file = output_file
        self.timestamp = datetime.datetime.now().isoformat()
        self.ros2 = ROS2Data()
        self.gazebo = GazeboData()
        self.console = Console()
    
    def _parse_ros2_topic_info(self, info_output: str) -> Dict[str, any]:
        lines = info_output.split('\n')
        
        publisher_count = next((int(line.split(':')[1].strip()) 
                               for line in lines if "Publisher count:" in line), 0)
        subscriber_count = next((int(line.split(':')[1].strip()) 
                                for line in lines if "Subscription count:" in line), 0)
        
        publishers = []
        subscribers = []
        current_section = None
        
        for line in lines:
            line = line.strip()
            if "Publishers:" in line:
                current_section = "publishers"
            elif "Subscriptions:" in line:
                current_section = "subscribers"
            elif line.startswith("Node name:"):
                node_name = line.split(':')[1].strip()
                if current_section == "publishers":
                    publishers.append(node_name)
                elif current_section == "subscribers":
                    subscribers.append(node_name)
        
        return {
            'publisher_count': publisher_count,
            'subscriber_count': subscriber_count,
            'publishers': publishers,
            'subscribers': subscribers
        }
    
    def _parse_gazebo_topic_info(self, info_output: str) -> Dict[str, any]:
        lines = info_output.split('\n')
        result = {'type': '', 'publishers': 0, 'subscribers': 0}
        
        for line in lines:
            if "Type:" in line:
                result['type'] = line.split(':', 1)[1].strip()
            elif "Publishers:" in line:
                try:
                    result['publishers'] = int(line.split(':')[1].strip().split()[0])
                except (ValueError, IndexError):
                    pass
            elif "Subscribers:" in line:
                try:
                    result['subscribers'] = int(line.split(':')[1].strip().split()[0])
                except (ValueError, IndexError):
                    pass
        
        return result
    
    def _get_ros2_topics(self) -> List[TopicInfo]:
        topic_names = safe_lines(run_command(["ros2", "topic", "list"]))
        topics = []
        
        for name in track(topic_names, description="[cyan]Processing ROS2 topics..."):
            topic_type = run_command(["ros2", "topic", "type", name]) or ""
            info_output = run_command(["ros2", "topic", "info", name]) or ""
            
            parsed_info = self._parse_ros2_topic_info(info_output)
            topics.append(TopicInfo(name=name, type=topic_type, **parsed_info))
        
        return topics
    
    def _get_ros2_services(self) -> List[ServiceInfo]:
        service_names = safe_lines(run_command(["ros2", "service", "list"]))
        
        services = []
        for name in track(service_names, description="[cyan]Processing ROS2 services..."):
            service_type = run_command(["ros2", "service", "type", name]) or ""
            services.append(ServiceInfo(name=name, type=service_type))
        
        return services
    
    def _get_ros2_nodes(self) -> List[str]:
        return safe_lines(run_command(["ros2", "node", "list"]))
    
    def _get_gazebo_topics(self) -> List[GazeboTopicInfo]:
        topic_names = safe_lines(run_command(["gz", "topic", "-l"]))
        topics = []

        for name in track(topic_names, description="[magenta]Processing Gazebo topics..."):
            info_output = run_command(["gz", "topic", "-i", "-t", name]) or ""
            parsed_info = self._parse_gazebo_topic_info(info_output)
            topics.append(GazeboTopicInfo(name=name, **parsed_info))

        return topics
    
    def _get_gazebo_services(self) -> List[ServiceInfo]:
        service_names = safe_lines(run_command(["gz", "service", "-l"]))
        services = []

        for name in track(service_names, description="[magenta]Processing Gazebo services..."):
            info_output = run_command(["gz", "service", "-i", "-s", name]) or ""
            lines = info_output.split('\n')
            
            request_type = ""
            response_type = ""
            
            for line in lines:
                if "Request type:" in line:
                    request_type = line.split(':', 1)[1].strip()
                elif "Response type:" in line:
                    response_type = line.split(':', 1)[1].strip()

            services.append(ServiceInfo(
                name=name,
                request_type=request_type,
                response_type=response_type
            ))
        
        return services
    
    def collect_ros2_data(self) -> None:
        self.console.print("[bold cyan]Collecting ROS2 data...[/bold cyan]")
        self.ros2.topics = self._get_ros2_topics()
        self.ros2.services = self._get_ros2_services()
        self.ros2.nodes = self._get_ros2_nodes()
    
    def collect_gazebo_data(self) -> None:
        self.console.print("[bold magenta]Collecting Gazebo data...[/bold magenta]")
        self.gazebo.topics = self._get_gazebo_topics()
        self.gazebo.services = self._get_gazebo_services()
    
    def save_to_yaml(self) -> None:
        data = {
            "timestamp": self.timestamp,
            "ros2": asdict(self.ros2),
            "gazebo": asdict(self.gazebo)
        }
        
        try:
            with open(self.output_file, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, indent=2)
            self.console.print(f"[green]Data saved to {self.output_file}[/green]")
        except Exception as e:
            self.console.print(f"[red]Error saving to file: {e}[/red]")
    
    def print_summary(self) -> None:
        self.console.print("\n" + "="*50)
        self.console.print("[bold]SUMMARY[/bold]")
        self.console.print("="*50)
        self.console.print(f"Timestamp: {self.timestamp}")
        
        ros2_topic_count = len(self.ros2.topics)
        ros2_service_count = len(self.ros2.services)
        ros2_node_count = len(self.ros2.nodes)
        gazebo_topic_count = len(self.gazebo.topics)
        gazebo_service_count = len(self.gazebo.services)
        
        if ros2_topic_count or ros2_service_count or ros2_node_count:
            self.console.print(f"\n[cyan]ROS2:[/cyan]")
            self.console.print(f"  Topics: {ros2_topic_count}")
            self.console.print(f"  Services: {ros2_service_count}")
            self.console.print(f"  Nodes: {ros2_node_count}")
        
        if gazebo_topic_count or gazebo_service_count:
            self.console.print(f"\n[magenta]Gazebo:[/magenta]")
            self.console.print(f"  Topics: {gazebo_topic_count}")
            self.console.print(f"  Services: {gazebo_service_count}")
        
        total = ros2_topic_count + ros2_service_count + ros2_node_count + gazebo_topic_count + gazebo_service_count
        self.console.print(f"\n[bold]Total entities tracked: {total}[/bold]")

def main():
    parser = argparse.ArgumentParser(description="ROS2 and Gazebo Topic Tracker")
    parser.add_argument("--ros2", action="store_true", help="Collect ROS2 data", default=True)
    parser.add_argument("--gazebo", action="store_true", help="Collect Gazebo data", default=False)
    parser.add_argument("-o", "--output", default="topic_tracking.yaml", help="Output file path")
    
    args = parser.parse_args()
    
    current_dir = os.path.dirname(os.path.abspath(__file__))
    output_file = args.output if os.path.isabs(args.output) else os.path.join(current_dir, args.output)
    
    tracker = TopicTracker(output_file)
    
    if args.ros2:
        tracker.collect_ros2_data()
    
    if args.gazebo:
        tracker.collect_gazebo_data()
    
    tracker.save_to_yaml()
    tracker.print_summary()

if __name__ == "__main__":
    main()