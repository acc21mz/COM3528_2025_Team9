#!/usr/bin/env python3

import tkinter as tk
import rospy
from std_msgs.msg import String
import os

class CommandWindow:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("MiRo Command Interface")
        
        self.robot_name = '/miro/'+os.getenv("MIRO_ROBOT_NAME")
        self.topic_root = self.robot_name
        self.pub = rospy.Publisher(self.topic_root + '/speech_to_text', String, queue_size=1)
    
        # Create command buttons
        self.create_buttons()
        
    def create_buttons(self):
        # Wake word buttons
        wake_frame = tk.LabelFrame(self.root, text="Wake Words", padx=5, pady=5)
        wake_frame.pack(padx=10, pady=5, fill="x")
        
        tk.Button(wake_frame, text="MiRo", command=lambda: self.send_command("miro")).pack(side=tk.LEFT, padx=5)
        
        # Action buttons
        action_frame = tk.LabelFrame(self.root, text="Commands", padx=5, pady=5)
        action_frame.pack(padx=10, pady=5, fill="x")
        
        commands = [
            ("Play Dead", "miro play dead"),
            ("Stop", "miro stop"),
            ("Fetch", "miro fetch"),
            ("Speak", "miro speak"),
        ]
        
        for text, cmd in commands:
            tk.Button(action_frame, text=text, 
                     command=lambda c=cmd: self.send_command(c)).pack(fill="x", padx=5, pady=2)
    
    def send_command(self, command):
        msg = String()
        msg.data = command.lower()
        self.pub.publish(msg)
        print(f"Sent command: {command}")
    
    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    try:
        rospy.init_node("miro_command_window")
        window = CommandWindow()
        window.run()
    except rospy.ROSInterruptException:
        pass