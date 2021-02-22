# Use Reachy to distribute masks! 

## Install everything

```bash
cd ~/dev
git clone https://github.com/pollen-robotics/reachy-masks.git
cd ~/dev/reachy-masks
pip3 install -e .
```

You'll need to make a few changes to *reachy API* to have this working.

To start the application:
```bash
cd ~/dev/reachy-masks
python3 -m reachy_masks.mask_launcher   
```

You will need to replace Reachy's left gripper with a custom 3d printed piece. You can find the 3d files [here](https://cad.onshape.com/documents/bab2997550deb4aad12c7153/w/db22f447bcbaf9fd28d5a02d/e/b4d43361b40b080d9f70675a).

## Add the following elements to Reachy

In `reachy/software/reachy/parts/head.py` add to the **__init__** function:
```python
self.previous_look_at = [0, 0, 0]
```
and to the **look_at** function:
```python
self.previous_look_at = [x, y, z]
```

Don't forget to create a new type of hand for the custom piece!
Declare it in `reachy/software/reachy/parts/hand.py`:
```python
class FlyerHand(Hand):
    """Half a wrist and no hand Part.

    Args:
        io (str): port name where the modules can be found
        side (str): which side the part is attached to ('left' or 'right')

    Composed of two dynamixel motors.
    """

    fans = {'wrist_fan': 'hand.wrist_pitch'}

    def __init__(self, root, io):
        """Create a new Empty Hand."""
        Hand.__init__(self, root=root, io=io)

        dxl_motors = OrderedDict({
            name: dict(conf)
            for name, conf in self.dxl_motors.items()
        })

        self.attach_dxl_motors(dxl_motors)
        
class LeftFlyerHand(FlyerHand):
    """Left Empty Hand Part."""

    dxl_motors = OrderedDict([
        ('forearm_yaw', {
            'id': 24, 'offset': 0.0, 'orientation': 'indirect',
            'angle-limits': [-100, 100],
            'link-translation': [0, 0, 0], 'link-rotation': [0, 0, 1],
        }),
        ('wrist_pitch', {
            'id': 25, 'offset': 0.0, 'orientation': 'indirect',
            'angle-limits': [-45, 70],
            'link-translation': [0, 0, -0.25], 'link-rotation': [0, 1, 0],
        }),
    ])
 ```
 and add it to the list of existing hands in `reachy/software/reachy/parts/arm.py`:
 ```python
 hands = {
    # [...] other hands already declared
    'flyer_hand': {'left': LeftFlyerHand},
}
```
