import cv2
import numpy as np
import time
import pickle

def current_milli_time():
    return round(time.time() * 1000)
model=cv2.dnn.readNetFromDarknet('models/yolo-tiny.cfg', 'models/yolov3-tiny.weights')
objth=0.5
confth=0.5
nmsth=0.5
inwidth=320
inheight=320
def getoutputlayers(model):
    layernames=model.getLayerNames()
    return [layernames[i[0]-1] for i in model.getUnconnectedOutLayers()]
def processndraw(frame, out, timer,algila):
    imghei = frame.shape[0]
    imgwid = frame.shape[1]
    classids = []
    confs = []
    boxes = []
    h1 = int(imghei * 0.05)
    h2 = int(imghei * 0.90)
    w1 = int(imgwid * 0.25)
    w2 = int(imgwid * 0.75)
    for o in out:
        for detected in o:
            if detected[5] > objth:
                scores = detected[5:]
                classid = np.argmax(scores)
                conf = scores[classid]
                if conf > confth:
                    center_x = int(detected[0] * imgwid)
                    center_y = int(detected[1] * imghei)
                    w = int(detected[2] * imgwid)
                    h = int(detected[3] * imghei)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    classids.append(classid)
                    confs.append(float(conf))
                    boxes.append([x, y, w, h])
    ind = cv2.dnn.NMSBoxes(boxes, confs, confth, nmsth)
    for i in ind:
        i = i[0]
        box = boxes[i]
        cframe = frame.copy()
        if algila == True:
             cv2.rectangle(frame, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]), (255, 255, 0), 3)
    if len(boxes) == 0:
        box = []
        cframe = frame.copy()
    return box,cframe,h1,w1,h2,w2
if __name__ == '__main__':
    drawing = False
    selected=False
    cap = cv2.VideoCapture(0)
    tracker=cv2.TrackerMOSSE_create()
    initialized=False
    bbox1=[]
    if not cap.isOpened():
        raise IOError("Cannot open webcam")
    width=cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height=cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    orta_x=int(width/2)
    orta_y=int(height/2)
    bbox=()
    axy=[orta_x,orta_y]
    # file_output='xdxd.avi'
    # fourcc=cv2.VideoWriter_fourcc(*'FMP4')
    # out=cv2.VideoWriter(file_output,fourcc,20.0,(int(width),int(height)))
    fp=open("pay1.pkl","wb")
    pickle.dump(axy,fp,protocol=2)
    fp.close()
    control = True
    kit = False
    seconds = 0
    sayac = 0
    algilama = True
    drone_w = 0
    drone_h = 0
    # veriler = {}
    s = 0
    lock = False
    speed = 0
    while True:
        zaman = time.strftime("%S")
        timer = cv2.getTickCount()
        ok, frame = cap.read()
        img = cv2.resize(frame, None, fx=1.2, fy=1.2, interpolation=cv2.INTER_AREA)
        blob = cv2.dnn.blobFromImage(img, 1 / 255, (inwidth, inheight), [0, 0, 0], 1, crop=False)
        model.setInput(blob)
        out = model.forward(getoutputlayers(model))
        xbox, frame, h1, w1, h2, w2 = processndraw(img, out, timer,algilama)
        if xbox != []:
            (x0, y0), (x1, y1) = (xbox[0], xbox[1]), (xbox[0] + xbox[2], xbox[1] + xbox[3])
        cv2.putText(img, "zamanlayici: " + str(seconds), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (0, 0, 255), 2)
        cv2.putText(img, "kitlenme: " + str(sayac), (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (0, 0, 255), 2)
        cv2.rectangle(img, (w1, h1), (w2, h2), (255, 0, 0), 2)
        print(s)
        print(xbox)
        if xbox != [] and s > 10:
            print("test")
            cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 255, 255), 2)
            print("algilama: ", x0,y0,x1,y1)
            bbox1 = [x0, y0, x1 - x0, y1 - y0]
            bbox = tuple(bbox1)
            tracked=tracker.init(img,bbox)
            kit = False
            print("xbox: ",xbox)
        if not ok:
            break
        ret,bbox=tracker.update(frame)
        print("xbox2: ",bbox)
        s += 1
        if ret:
            p1= (int(bbox[0]),int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            drone_w = p2[0] - p1[0]
            drone_h = p2[1] - p1[1]
            print("değer: ",p1,p2)
            cv2.rectangle(img,p1,p2,(0,0,255),2,1)
            algilama = False
            a_x=int(2*bbox[0]+bbox[2])/2
            a_y=int(2*bbox[1]+bbox[3])/2
            axy=[a_x,a_y]
            cv2.arrowedLine(img, (320, 480), (int(a_x),int(a_y)), (255, 255, 255), 2)
            if p1[0] < w1 or p1[1] < h1 or p2[0] > w2 or p2[1] > h2:
                seconds = 0
                # iha sağ,sol, üst, alt yönüne hareket edecek
            else:
                if control == True:
                    Times = int(zaman)
                    control = False
                    # print("ekran boyutu: ",int(img.shape[0] * 0.15),int(img.shape[1] * 0.15))
                    # print("drone: ",drone_h,drone_w)
                    print()
                if int(zaman) == Times + 1 and kit == False:
                    if drone_h > int(img.shape[0] * 0.05) or drone_w > int(img.shape[1] * 0.05):
                            print(zaman)
                            seconds += 1
                            control = True
                            if seconds == 10:  # kitlenme gerçekleşti
                                d1 = p1
                                d2 = p2
                                print(d1,d2)
                                kit = True
                                algilama = True
                                sayac += 1
                                seconds = 0
                                s = 0
                                lock = True

            if drone_h > int(img.shape[0] * 0.08) or drone_w > int(img.shape[1] * 0.08):
                pass
        else:
            axy=[orta_x,orta_y]
        print(axy)
        fp=open("pay1.pkl","wb")
        pickle.dump(axy,fp,protocol=2)
        fp.close()
        cv2.imshow('Webcam', img)
        c = cv2.waitKey(5)
        if c == 27:
            axy=[orta_x,orta_y]
            fp=open("pay1.pkl","wb")
            pickle.dump(axy,fp,protocol=2)
            fp.close()
            # print(axy)
            time.sleep(0.5)
            break
        timer2 = cv2.getTickCount()
        t = (timer2 - timer) / cv2.getTickFrequency()
        print("geçen zaman: ", t)
    cap.release()
    out.release()
    cv2.destroyAllWindows()
