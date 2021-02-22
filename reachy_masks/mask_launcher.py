import logging
import time

from reachy import Reachy, parts
from . import MaskBackground

import zzlog

logger = logging.getLogger('reachy.flyers')


def run_distribution_loop(mask_background):

    tracking_threshold = 20*20
    action_threshold = 150*150

    track_count = 0
    give_count = 0
    action_count = 0

    flyer_given = 0

    was_alone = True
    was_in_idle_mode = False

    mask_background.detection.start()
    mask_background.activate_tracking_mode()
    time.sleep(0.02)

    while True:

        if mask_background.detection._somebody_detected:

            logger.info('Someone has been detected')

            mask_background.idleForever.stop()
            if was_in_idle_mode:
                mask_background.reinitialize_target()
                was_in_idle_mode = False
            mask_background.get_target_info()

            # if someone is close enough to begin an interaction 
            if mask_background.target_size > tracking_threshold:
                track_count = 0

                # if someone is close, but not too close 
                if mask_background.target_size < action_threshold:

                    logger.info('Person is not close enough, tracking mode activated',
                                extra={'target-size': mask_background.target_size})

                    mask_background.activate_tracking_mode()
                    mask_background.track()

                    give_count = 0
                    action_count = 0

                    # flyer_background.detect_new_person(was_alone)
                    was_alone = False

                # if someone is close enough to start giving the flyer 
                else:

                    if mask_background.person_comes_for_flyer():

                        mask_background.deactivate_tracking_mode()

                        # is the person has not been seen before
                        if mask_background.is_new_person():

                            # give count used to prevent Reachy from giving flyers too frequently 
                            if give_count == 0:

                                logger.info('Reachy is ready to give a flyer.')
                                mask_background.take_flyer_modified()

                                track = 0

                                mask_background.activate_tracking_mode()

                                # after pulling the flyer, track a bit in case the person has moved
                                while track < 2:
                                    if mask_background.detection._somebody_detected:
                                        mask_background.get_target_info()
                                        mask_background.track()
                                        track += 1
                                        time.sleep(0.02)
                                    else:
                                        mask_background.look_at_previous_target()

                                mask_background.deactivate_tracking_mode()
                                mask_background.give_flyer()

                                flyer_given += 1
                                logger.info('Reachy has given the flyer',
                                            extra={
                                                'flyer_number': flyer_given,
                                            }
                                            )

                                give_count = 40

                            give_count -= 1
                            time.sleep(0.1)
                            mask_background.activate_tracking_mode()

                        else:
                            if action_count == 0:
                                mask_background.no_flyer()
                                action_count = 20
                            action_count -= 1
                            mask_background.activate_tracking_mode()

                    # track at the end in all cases
                    mask_background.get_target_info()
                    mask_background.track()

        else:
            if track_count < 40:
                track_count += 1
            else:
                logger.info('No one detected, Reachy plays random behavior.')
                track_count = 0
                mask_background.deactivate_tracking_mode()
                time.sleep(0.2)

                mask_background.idleForever.start()
                was_alone = True
                was_in_idle_mode = True

        time.sleep(0.02)


if __name__ == '__main__':

    import argparse

    from datetime import datetime
    from glob import glob

    parser = argparse.ArgumentParser()
    parser.add_argument('--log-file')
    args = parser.parse_args()

    if args.log_file is not None:
        n = len(glob(f'{args.log_file}*.log')) + 1

        now = datetime.now().strftime('%Y-%m-%d_%H:%M:%S.%f')
        args.log_file += f'-{n}-{now}.log'

    _ = zzlog.setup(
        logger_root='',
        filename=args.log_file,
    )

    logger.info(
        'Initializing flyer distribution.'
    )

    with MaskBackground() as mask_background:
        mask_background.setup()
        run_distribution_loop(mask_background)
