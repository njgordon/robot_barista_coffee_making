#!/usr/bin/env python
import rospy
from std_msgs.msg import ColorRGBA, Float32
# sound imports
from sound_play.libsoundplay import SoundClient
try:
  from jsk_rviz_plugins.msg import *
except:
  import roslib;roslib.load_manifest("jsk_rviz_plugins")
  from jsk_rviz_plugins.msg import *

# contains all Fetch primitives related sound (Fetch's speaker)
class FetchSound(object):

    # voice options to select for Fetch
    # first param is sound_play name
    # second param is name of matching file to 'sudo apt-get install'
    VOICE_OPTIONS = [
        ['voice_kal_diphone', 'festvox-kallpc16k'] #default, comes pre-installed
        , ['voice_don_diphone', 'festvox-don'] #aweful
        , ['voice_ked_diphone', 'festvox-kdlpc16k'] #good
        , ['voice_rab_diphone', 'festvox-rablpc16k'] #good
        , ['voice_us1_mbrola', 'festvox-us1'] #best
        , ['voice_us2_mbrola', 'festvox-us2'] #good
        , ['voice_us3_mbrola', 'festvox-us3'] #aweful
    ]

    # adapted from jsk-visualization overlay_sample.py
    def __init__(self):
        # configuration for RVIZ text overlay
        self.text_pub = rospy.Publisher("fetch_speech_text", OverlayText, queue_size=1)
        self.rviz_text_overlay = OverlayText()
        self.rviz_text_overlay.width = 600
        self.rviz_text_overlay.height = 450
        self.rviz_text_overlay.left = 300
        self.rviz_text_overlay.top = 100
        self.rviz_text_overlay.text_size = 20
        self.rviz_text_overlay.line_width = 2
        self.rviz_text_overlay.font = "DejaVu Sans Mono"
        self.rviz_text_overlay.fg_color = ColorRGBA(25 / 255.0, 170 / 255.0, 255.0 / 255.0, 1.0) # Roughly matches Fetch's Blue
        self.rviz_text_overlay.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.0)

        # uses sound_play library for text-to-speech and playing sounds on Fetch
        self.soundhandle = SoundClient()
        # This pause is critical, otherwise the subscriber misses the handshake
        rospy.sleep(0.5)

        # Voice selection: If you only have the default installed use VOICE_OPTIONS[0][0]
        self.voice = self.VOICE_OPTIONS[4][0]
    
    def talk(self,text):
            paramaters = {'text': text,'volume':1.0}
            self.speak(paramaters)

    # Primitive: speak - say the words in "text"
    def speak(self, parameters={}):
        # "text" string to use for text-to-speech
        try:
            text = str(parameters["text"])
        except (ValueError, KeyError) as e:
            text = "Hello"

        # "volume" of sound from 0.0 to 1.0
        try:
            volume = float(parameters["volume"])
        except (ValueError, KeyError) as e:
            volume = 1.0
        if volume > 1.0 or volume < 0.0:
            volume = 1.0

        # send sound file to Fetch hardware
        self.soundhandle.say(text, voice = self.voice, volume = volume)

        # send text to RVIZ overlay
        # Note: adding in css to the string will format it further in RVIZ
        self.rviz_text_overlay.text = text
        self.text_pub.publish(self.rviz_text_overlay)

        # give Fetch some time to talk, so the sound doesn't get blocked by the next action
        pause_time = 0.2*len(text)
        rospy.sleep(pause_time)
        self.rviz_text_overlay.text = ''
        self.text_pub.publish(self.rviz_text_overlay)

        rospy.loginfo("Fetch says: '%s'", text)

    # Primitive: stop_sound
    def stop_sound(self, parameters={}):
        self.speak(parameters={"text": ''})
