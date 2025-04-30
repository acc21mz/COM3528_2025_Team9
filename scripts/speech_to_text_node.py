#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import speech_recognition as sr
import command_recognition_sim as command_rec

def listen_and_publish():
    rospy.init_node('speech_to_text_node', anonymous=True)
    pub = rospy.Publisher('/speech_to_text', String, queue_size=10)

    recognizer = sr.Recognizer()
    mic = sr.Microphone()

    rospy.loginfo("🎤 Speech-to-Text node started. Listening...")

    with mic as source:
        recognizer.adjust_for_ambient_noise(source)

    while not rospy.is_shutdown():
        with mic as source:
            print("Say something:")
            audio = recognizer.listen(source)

        try:
            text = recognizer.recognize_google(audio)
            print(f"✅ You said: {text}")
            rospy.loginfo(f"[STT] Publishing: {text}")
            pub.publish(String(data=text))
            command_rec.CommandRecognition().callback_receive_command(text)
        except sr.UnknownValueError:
            print("❌ Could not understand audio.")
        except sr.RequestError as e:
            print(f"❌ Could not request results from Google Speech Recognition service; {e}")

if __name__ == '__main__':
    try:
        listen_and_publish()
    except rospy.ROSInterruptException:
        pass
