import time
import struct
import numpy as np
import cv2
from multiprocessing import shared_memory, resource_tracker
from infer_wrap import InferWrap

SHM_NAME = "shm_ar_video"
SHM_HEADER_SIZE = 16

#detecrion init
infer = InferWrap(TPEs=3)

# ¡¾¹Ø¼üÐÞ¸´¡¿·ÀÖ¹¿Í»§¶ËÍË³öÊ±ÎóÉ¾·þÎñÆ÷µÄ SHM
def remove_shm_from_resource_tracker():
    try:
        resource_tracker.unregister('/' + SHM_NAME, 'shared_memory')
    except:
        pass

def main():
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
                    if fid == last_fid:
                        time.sleep(0.002)
                        if cv2.waitKey(1) == 27: raise KeyboardInterrupt
                        continue

                    last_fid = fid
                    
                    # 3. ¶ÁÈ¡Êý¾Ý
                    size = w * h * 3
                    # Ö±½ÓÉî¿½±´µ½±¾µØ£¬±ÜÃâ³¤Ê±¼äÕ¼ÓÃ SHM
                    img_view = np.ndarray((h, w, 3), dtype=np.uint8, buffer=shm.buf[SHM_HEADER_SIZE : SHM_HEADER_SIZE+size])
                    frame = img_view.copy() # Deep Copy
                    del img_view # Á¢¼´ÊÍ·ÅÊÓÍ¼
                    
                    # 4. ÏÔÊ¾´¦Àí
                    frame = cv2.flip(frame, 0)
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    
                    # FPS
                    fps_n += 1
                    if time.time() - fps_t >= 1.0:
                        cur_fps = fps_n / (time.time() - fps_t)
                        fps_n = 0; fps_t = time.time()
                    
                    cv2.putText(frame, f"Client FPS: {cur_fps:.1f}", (10, 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    # cv2.imshow("AR Client", frame)
                    det_frame, flag = infer.infer(frame)
                    cv2.imshow("ret", det_frame)
                    
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