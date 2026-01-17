from ultralytics import YOLO


def train_model(data_path, epochs=50):
    model = YOLO('yolov8n.pt')  # nano — najmniejszy, szybki model
    model.train(
        data=data_path,
        epochs=epochs,
        imgsz=512,         # trochę mniejszy niż 640 — lepszy balans
        batch=8,           # bezpieczna wartość dla 6GB VRAM
        device=0,          # GPU
        workers=2,         # unikamy zbyt wielu wątków
        cache=False,       # nie ładuje wszystkiego do RAM
        project='yolo_thermal',
        name='person_detection',
        exist_ok=True,
        verbose=True
    )


def test_model(model_path, test_images):
    model = YOLO(model_path)
    results = model.predict(source=test_images, conf=0.25, save=True)
    print("Predictions saved.")

if __name__ == "__main__":
    
    # 1. Trenowanie modelu
    #train_model(data_path='config/data.yaml', epochs=50)
    
    # 2. Testowanie modelu
    test_model(model_path='/home/mateusz/Desktop/sem2_magisterka/drony/drone_night_patrol/yolo_thermal/person_detection/weights/best.pt', test_images='/home/mateusz/Desktop/sem2_magisterka/drony/drone_night_patrol/src/drone_sim/resource/infrared.v1-test.yolov8/test/images')
