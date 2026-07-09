import os
import sys
import time
import struct
import numpy as np
import cv2
from multiprocessing import shared_memory, resource_tracker

# ====== DIAGNOSTICS: print environment so we can debug display issues ======
print("=" * 60)
print(f"[AR_RECEIVER DIAG] PID={os.getpid()}")
print(f"[AR_RECEIVER DIAG] DISPLAY={os.environ.get('DISPLAY', 'NOT SET')}")
print(f"[AR_RECEIVER DIAG] XAUTHORITY={os.environ.get('XAUTHORITY', 'NOT SET')}")
print(f"[AR_RECEIVER DIAG] QT_X11_NO_MITSHM={os.environ.get('QT_X11_NO_MITSHM', 'NOT SET')}")
print(f"[AR_RECEIVER DIAG] LD_LIBRARY_PATH={os.environ.get('LD_LIBRARY_PATH', 'NOT SET')}")
print(f"[AR_RECEIVER DIAG] PYTHONPATH={os.environ.get('PYTHONPATH', 'NOT SET')}")
print(f"[AR_RECEIVER DIAG] CWD={os.getcwd()}")
print(f"[AR_RECEIVER DIAG] sys.executable={sys.executable}")
print(f"[AR_RECEIVER DIAG] OpenCV build info (backend): {cv2.getBuildInformation().split('GUI')[0].split('Video')[-1][:200] if 'GUI' in cv2.getBuildInformation() else 'N/A'}")
print("=" * 60)
sys.stdout.flush()

SHM_NAME = "shm_ar_video"
SHM_HEADER_SIZE = 16

infer = None
try:
    from infer_wrap import InferWrap
    infer = InferWrap(TPEs=3)
    print("[AR_RECEIVER] infer_wrap enabled.")
except Exception as e:
    print(f"[AR_RECEIVER] infer_wrap unavailable, preview-only mode: {e}")
sys.stdout.flush()

# ¡¾¹Ø¼üÐÞ¸´¡¿·ÀÖ¹¿Í»§¶ËÍË³öÊ±ÎóÉ¾·þÎñÆ÷µÄ SHM
def remove_shm_from_resource_tracker():
    try:
        resource_tracker.unregister('/' + SHM_NAME, 'shared_memory')
    except:
        pass

def main():
    global infer
    print("?? Client Ready. Waiting for Server...")
    
    while True: # ¶ÏÏßÖØÁ¬Ñ­»·
        shm = None
        try:
            # 1. ³¢ÊÔÁ¬½Ó
            try:
                shm = shared_memory.SharedMemory(name=SHM_NAME)
                # Á¬½Ó³É¹¦ºó£¬Á¢¿Ì´ÓÇåÀíÃûµ¥ÖÐÒÆ³ý
                remove_shm_from_resource_tracker()
                print("? Connected!")
            except FileNotFoundError:
                time.sleep(1.0)
                continue

            last_fid = 0
            
            # FPS Í³¼Æ
            fps_t = time.time()
            fps_n = 0
            cur_fps = 0.0

            while True:
                try:
                    # 2. ¶ÁÈ¡Í·²¿
                    header = bytes(shm.buf[:SHM_HEADER_SIZE])
                    fid, w, h = struct.unpack('QII', header)
                    
                    # ÓÅ»¯£ºÖ¡ºÅÃ»±ä¾Í²»¶Á
                    if fid == 0 or fid == last_fid:
                        time.sleep(0.002)
                        if cv2.waitKey(1) == 27: raise KeyboardInterrupt
                        continue

                    # 3. ¶ÁÈ¡Êý¾Ý
                    size = w * h * 3
                    # Ö±½ÓÉî¿½±´µ½±¾µØ£¬±ÜÃâ³¤Ê±¼äÕ¼ÓÃ SHM
                    img_view = np.ndarray((h, w, 3), dtype=np.uint8, buffer=shm.buf[SHM_HEADER_SIZE : SHM_HEADER_SIZE+size])
                    frame = img_view.copy() # Deep Copy
                    del img_view # Á¢¼´ÊÍ·ÅÊÓÍ¼
                    header2 = bytes(shm.buf[:SHM_HEADER_SIZE])
                    fid2, w2, h2 = struct.unpack('QII', header2)
                    if fid2 != fid or w2 != w or h2 != h:
                        continue
                    last_fid = fid
                    
                    # 4. ÏÔÊ¾´¦Àí
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    
                    # FPS
                    fps_n += 1
                    if time.time() - fps_t >= 1.0:
                        cur_fps = fps_n / (time.time() - fps_t)
                        fps_n = 0; fps_t = time.time()
                    
                    cv2.putText(frame, f"Client FPS: {cur_fps:.1f}", (10, 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    if infer is not None:
                        try:
                            det_frame, _ = infer.infer(frame)
                            cv2.imshow("AR Preview", det_frame)
                        except Exception as e:
                            print(f"[AR_RECEIVER] inference disabled after error: {e}")
                            infer = None
                            cv2.imshow("AR Preview", frame)
                    else:
                        cv2.imshow("AR Preview", frame)
                    
                    if cv2.waitKey(1) == 27: raise KeyboardInterrupt

                except (ValueError, struct.error, BufferError):
                    raise FileNotFoundError # ÊÓÎªÁ¬½Ó¶Ï¿ª

        except KeyboardInterrupt:
            print("\n?? Client Stop.")
            break
        except FileNotFoundError:
            print("?? Connection lost... Waiting...")
            if shm: shm.close()
            cv2.destroyAllWindows()
            time.sleep(1.0)
        finally:
            if shm: 
                try: shm.close()
                except: pass

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
