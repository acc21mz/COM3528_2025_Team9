#!/usr/bin/env python3

import tkinter as tk
import rospy
from std_msgs.msg import String

class CommandWindow:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("MiRo Command Interface")
        
        # Initialize ROS node and publisher
        rospy.init_node('command_gui', anonymous=True)
        self.pub = rospy.Publisher('/miro/sim01/speech_to_text', String, queue_size=1)
        
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
            ("Sleep", "sleep"),
            ("Bad", "bad"),
            ("Play", "play"),
            ("Follow me", "follow me"),
            ("Good", "good"),
            ("Speak", "speak"),
            ("Fetch", "fetch"),
            ("Stop", "stop")
        ]
        
        for text, cmd in commands:
            tk.Button(action_frame, text=text, 
                     command=lambda c=cmd: self.send_command(c)).pack(fill="x", padx=5, pady=2)
    
    def send_command(self, command):
        msg = String()
        msg.data = command
        self.pub.publish(msg)
        print(f"Sent command: {command}")
    
    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    try:
        window = CommandWindow()
        window.run()
    except rospy.ROSInterruptException:
        pass