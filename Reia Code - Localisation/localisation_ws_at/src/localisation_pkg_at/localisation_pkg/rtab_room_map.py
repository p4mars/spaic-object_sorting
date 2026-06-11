'''
IGNORE THIS CODE - NOT MEANT FOR MIRTE
THIS CODE WAS PURELY USED TO TEST THE WORKING OF RTAB MAP PARAMS BY USING MY WEBCAM TO MAP MY ROOM
i kept the file just incase
'''




import cv2
import numpy as np
import pickle
import time

#  settings
MAP_FILE = 'room_map.pkl'
MODE     = 'map'   # change to 'localise' after mapping

# initialise
cap = cv2.VideoCapture(1)
orb = cv2.ORB_create(nfeatures=1000)
bf  = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# each keyframe stores: image, keypoints, descriptors, position
map_keyframes = []
position_x    = 0.0
position_y    = 0.0
frame_count   = 0

# ......................................................................................................................
#                                                     LOCALISATION
#.......................................................................................................................
if MODE == 'localise':
    print('Loading map...')
    with open(MAP_FILE, 'rb') as f:
        map_keyframes = pickle.load(f)
    print(f'Map loaded: {len(map_keyframes)} keyframes')

print(f'Mode: {MODE.upper()}')
if MODE == 'map':
    print('Move camera slowly around your room')
    print('Press S to save a keyframe')
    print('Press F to finish and save map')
else:
    print('Move camera to different positions')
    print('System will guess your position')
print('Press Q to quit')


def keypoints_to_list(kps):
    return [(k.pt, k.size, k.angle, k.response,
             k.octave, k.class_id) for k in kps]


def list_to_keypoints(kps_list):
    return [cv2.KeyPoint(
        x=k[0][0], y=k[0][1], size=k[1],
        angle=k[2], response=k[3],
        octave=k[4], class_id=k[5]
    ) for k in kps_list]


while True:
    ret, frame = cap.read()
    if not ret:
        break

    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    keypoints, descriptors = orb.detectAndCompute(grey, None)

    display = frame.copy()

    # draw features
    if keypoints:
        cv2.drawKeypoints(
            display, keypoints, display,
            color=(0, 255, 0),
            flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
        )

# ......................................................................................................................
#                                                     MAPPING
#.......................................................................................................................
    if MODE == 'map':

        cv2.putText(display,
            f'MAPPING MODE  |  keyframes: {len(map_keyframes)}',
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
            0.7, (0, 255, 0), 2)
        cv2.putText(display,
            'S = save keyframe  |  F = finish  |  Q = quit',
            (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
            0.6, (255, 255, 0), 1)
        cv2.putText(display,
            f'Features: {len(keypoints)}',
            (10, 90), cv2.FONT_HERSHEY_SIMPLEX,
            0.6, (0, 255, 0), 1)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('s') and descriptors is not None:
            # save current frame as keyframe
            map_keyframes.append({
                'image':       frame.copy(),
                'keypoints':   keypoints_to_list(keypoints),
                'descriptors': descriptors.copy(),
                'position_x':  float(len(map_keyframes)),
                'position_y':  0.0,
                'label':       f'keyframe_{len(map_keyframes)}'
            })
            print(f'Keyframe {len(map_keyframes)} saved'
                  f' — {len(keypoints)} features')

            # flash green to confirm save
            flash = np.zeros_like(frame)
            flash[:] = (0, 255, 0)
            cv2.addWeighted(display, 0.5, flash, 0.5, 0, display)

        elif key == ord('f'):
            # save map to file
            save_data = []
            for kf in map_keyframes:
                save_data.append({
                    'image':       kf['image'],
                    'keypoints':   kf['keypoints'],
                    'descriptors': kf['descriptors'],
                    'position_x':  kf['position_x'],
                    'position_y':  kf['position_y'],
                    'label':       kf['label']
                })
            with open(MAP_FILE, 'wb') as f:
                pickle.dump(save_data, f)
            print(f'Map saved to {MAP_FILE}')
            print(f'Total keyframes: {len(map_keyframes)}')
            print('Change MODE to localise and rerun')
            break

        elif key == ord('q'):
            break
# ......................................................................................................................
#                                                     LOCALISATION
# .......................................................................................................................
    else:
        best_match_count = 0
        best_keyframe    = None
        best_matches     = []
        best_kps         = []

        if descriptors is not None and len(map_keyframes) > 0:
            for kf in map_keyframes:
                kf_kps  = list_to_keypoints(kf['keypoints'])
                kf_desc = kf['descriptors']

                if kf_desc is None or len(kf_desc) == 0:
                    continue

                try:
                    matches = bf.match(descriptors, kf_desc)
                    matches = sorted(
                        matches, key=lambda x: x.distance
                    )
                    good = [m for m in matches if m.distance < 60]

                    if len(good) > best_match_count:
                        best_match_count = len(good)
                        best_keyframe    = kf
                        best_matches     = good
                        best_kps         = kf_kps
                except:
                    continue

        if best_keyframe is not None and best_match_count > 10:
            # found a match — show position
            px = best_keyframe['position_x']
            py = best_keyframe['position_y']
            label = best_keyframe['label']

            confidence = min(100, best_match_count * 2)

            cv2.putText(display,
                f'LOCALISING',
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (0, 255, 0), 2)
            cv2.putText(display,
                f'Position: keyframe {int(px)}  ({label})',
                (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (0, 255, 255), 2)
            cv2.putText(display,
                f'Matches: {best_match_count}  '
                f'Confidence: {confidence}%',
                (10, 90), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 200, 255), 1)

            # draw match lines on reference keyframe
            ref_img = best_keyframe['image'].copy()
            match_display = cv2.drawMatches(
                frame, keypoints,
                ref_img, best_kps,
                best_matches[:20], None,
                flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
            )
            cv2.putText(match_display,
                f'Matched to: {label}  '
                f'({best_match_count} features)',
                (10, match_display.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 255, 255), 2)
            cv2.imshow('Feature Matches', match_display)

        else:
            cv2.putText(display,
                'LOST — not enough matches',
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (0, 0, 255), 2)
            cv2.putText(display,
                f'Best: {best_match_count} matches (need >10)',
                (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 0, 255), 1)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cv2.imshow('Camera', display)

cap.release()
cv2.destroyAllWindows()
