from ultralytics import YOLO

def __main__():
    # Load pretrained model (you have yolo26n.pt)
    model = YOLO("yolo26m.pt")

    # # Train
    model.train(
        data="data.yaml",
        epochs=600, #itterations
        imgsz=960, #image size
        batch= 16,
        amp=True,
        device=0,
        workers=8, #sets cpu cores
        patience=50, #Makes it stop if the same results are given for 50 epochs in a row
        optimizer="AdamW",
        cache=False, #doesn't keep cache the whole run
        auto_augment= None, #turned off because it overwrites set augmenation
        multi_scale=0.2, #applies a 20% increase of decrease in image size at random
        lr0 = 0.001, #learning rate
        cos_lr=True, #uses cos curve for the learning rate change
        mosaic=0.4, #creates a mosaic 40% of the time that includes more pictures and thus features
        close_mosaic=0, #does not disable mosaic for last optimization steps
        degrees=10, #applies rotation range for images
        scale=0.2, #applies scale range
        fliplr=0.5, #applies 50% chance for an image to be flipped
        hsv_h=0.015, #applies change in hue to create more data
        hsv_s=0.7, #applies change of saturation to create more data
        hsv_v=0.4, #applies change in brightness to create more data
        copy_paste=0.5, #applies copy paste 90% of the time
        copy_paste_mode='flip', #sets features from other images to be copied to other images and empty images
        cutmix = 0.2, #pastes a square cutout from one image with a feature into another
        translate = 0.1, #translates the image by 10% to perhaps spread features more around the space
        cls=1.5, #weight of class errors, makes classes more important than boxes
        box=5 #reduced box weight
    )

    # Validate
    metrics = model.val()
    print(metrics)

if __name__ == "__main__":
    __main__()