from picozero import Speaker

speaker = Speaker(13)
BEAT = 0.4

def c_note():
    speaker.play('c4', 0.5) # play the middle c for half a second

def d_note():
    speaker.play('d4', 0.5) # play the middle d for half a second

def d2_note():
    speaker.play('d2', 0.5) # play the middle d for half a second