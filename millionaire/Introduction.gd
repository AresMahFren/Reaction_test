extends Control

# References to your nodes - make sure these names match your Scene tree!
@onready var video = $Start # In your screenshot this is named "Start"
@onready var profile_image = $Play
@onready var menu = $Choose # In your screenshot this is named "Choose"

var can_press_enter = false

func _ready():
	# 1. Hide the stuff we don't need yet
	profile_image.hide()
	menu.hide()
	
	# 2. Start the video
	video.play()
	
	# 3. Listen for the video ending
	video.finished.connect(_on_video_finished)

func _on_video_finished():
	# 4. Video ended, show the profile image (the one with your photo)
	video.hide()
	profile_image.show()
	can_press_enter = true

func _input(event):
	# 5. Wait for Enter key while the photo is visible
	if can_press_enter and event.is_action_pressed("ui_accept"):
		show_mode_selection()

func show_mode_selection():
	# 6. Hide the photo/prompt and show the "Choose" texture
	can_press_enter = false
	profile_image.hide() 
	menu.show() 
	print("Ready to choose mode!")
