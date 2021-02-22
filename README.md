# Use Reachy to distribute masks! 

Install everything

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
