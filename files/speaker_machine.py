from picozero import Speaker

speaker = Speaker(13)
BEAT = 0.4

def c_note():
    speaker.play('c4', 0.5) # play the middle c for half a second