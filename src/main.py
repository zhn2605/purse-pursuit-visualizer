from current import Current as Car
from purePursuit import PurePursuit
from history import History
from history import Track

history = History()
history.generate_track()

car = Car(velocity=1.0)
pure_pursuit = PurePursuit()

history.animate(car, pure_pursuit)
