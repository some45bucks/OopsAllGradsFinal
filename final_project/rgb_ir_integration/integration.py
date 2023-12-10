from ultralytics import YOLO

def integrated_predictions(ir_model=None, rgb_model=None, ir_img=None, rgb_img=None, display=False):  
  ir_idx_list = []
  rgb_idx_list = []

  if ir_model is not None and ir_img is not None:
    ir_result = ir_model(ir_img)

    ir_names = ir_result[0].names
    ir_boxes = ir_result[0].boxes

    ir_found_classes = ir_boxes.cls
    ir_found_pobabilites = ir_boxes.conf
    ir_found_xyxy = ir_boxes.xyxy

    for index, value in enumerate(ir_found_classes.tolist()):
      if display:
        print(f'ir | {ir_names[int(value)]}: {ir_found_pobabilites[int(index)]}')
      if ir_names[int(value)] == 'person':
        ir_idx_list.append(int(index))

  if rgb_model is not None and rgb_img is not None:
    rgb_result = rgb_model(rgb_img)

    rgb_names = rgb_result[0].names
    rgb_boxes = rgb_result[0].boxes

    rgb_found_classes = rgb_boxes.cls
    rgb_found_pobabilites = rgb_boxes.conf
    rgb_found_xyxy = rgb_boxes.xyxy

    for index, value in enumerate(rgb_found_classes.tolist()):
      if display:
        print(f'rgb| {rgb_names[int(value)]}: {rgb_found_pobabilites[int(index)]}')
      if rgb_names[int(value)] == 'person':
        rgb_idx_list.append(int(index))
        

  for i in range(max(len(ir_idx_list), len(rgb_idx_list))):
    if display:
      print(len(ir_idx_list), len(rgb_idx_list))
    ir_idx = None
    ir_prob = None
    ir_xyxy = None
    rgb_idx = None
    rgb_prob = None
    rgb_xyxy = None
    if i < len(ir_idx_list):
      ir_idx  = ir_idx_list[i]
      ir_prob = ir_found_pobabilites[ir_idx]
      ir_xyxy = ir_found_xyxy[ir_idx]
    if i < len(rgb_idx_list):
      rgb_idx  = rgb_idx_list[i]
      rgb_prob = rgb_found_pobabilites[rgb_idx]
      rgb_xyxy = rgb_found_xyxy[rgb_idx]

    if ir_prob is None:
      ir_prob = 0
    if rgb_prob is None:
      rgb_prob = 0
    
    integrated_prob = max(ir_prob, rgb_prob)
    if display:
      print(f'(ir, rgb, max): ({ir_prob}, {rgb_prob}, {integrated_prob})')
    return integrated_prob 

if __name__ == '__main__':
  print('loading ir model')
  ir_model = YOLO('best_ir.pt')
  print('loading rgb model')
  rgb_model = YOLO('yolov8n.pt')
  integrated_predictions(ir_model=ir_model, rgb_model=rgb_model, ir_img='video-ZAtDSNuZZjkZFvMAo-frame-000365-SziuCPsGZWuTkA7q2.jpg', rgb_img='video-RMxN6a4CcCeLGu4tA-frame-000365-s9XeZQNmPyW483SWv.jpg', display=True)