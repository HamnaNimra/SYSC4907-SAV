#!/usr/bin/env python3
import tkinter as tk
from enum import Enum
import MapSerializer as MS
import PIL
from PIL import ImageTk, Image
from Models import RoadSegment, RoadSegmentType, MapModel, Lane, Connection, Path, Point
import os
from typing import Union, Tuple, List


class Mode(Enum):
    POINT = 0
    PATH = 1


op_mode: Mode = Mode.POINT


# Set the operation mode
def set_mode(mode: Mode):
    global op_mode
    op_mode = mode


# id of selected element (segment/lane/path)
selected_seg_id: int = -1
selected_lane_id: int = -1
selected_path_id: int = -1
map_model: MapModel = MapModel()


# Handle load from file
def handle_load():
    global map_model
    map_model = MS.load_from_file()
    # Update render
    rerender_points()
    rerender_road_segments_panel()
    rerender_paths_panel()


# Add point if lane is selected
def handle_canvas_m1(event):
    x, y = event.x, event.y

    if op_mode == Mode.POINT:
        if selected_seg_id != -1 and selected_lane_id != -1:
            # Draw canvas point
            obj_id: int = draw_point(x, y, (f's{selected_seg_id}', f'l{selected_lane_id}'))
            # Update model
            map_model.road_segments[selected_seg_id].add_point(selected_lane_id, Point(x, y, obj_id))
            rerender_road_segments_panel()
    elif op_mode == Mode.PATH:
        overlap_size: int = 3
        if selected_path_id != -1:
            # Get point_id and coordinates of point clicked
            found_ids: Tuple[int] = canvas.find_overlapping(x-overlap_size, y-overlap_size,
                                                            x+overlap_size, y+overlap_size)
            if len(found_ids) < 2:
                # Didn't click a point
                return

            point_id: int = found_ids[-1]
            coords = canvas.coords(point_id)
            x = (coords[0] + coords[2])/2
            y = (coords[1] + coords[3])/2

            clicked_point = Point(x, y, point_id)
            # Get segment id
            seg_id, lane_id = get_point_data(point_id)

            sel_path = map_model.paths[selected_path_id]
            if sel_path.empty():
                # Connection is None for first point
                sel_path.add_to_path(Connection(from_seg_id=seg_id, from_point=clicked_point),
                                     clicked_point)
            else:
                # Get the last connection, point on the path
                prev_con, prev_point = sel_path.connections[-1]
                sel_path.add_to_path(Connection(from_seg_id=prev_con.to_seg_id, from_point=prev_point,
                                                to_seg_id=seg_id, to_point=clicked_point), clicked_point)

            rerender_paths_panel()
            highlight_path()
    else:
        print('Invalid mode')


# Delete point
def handle_canvas_m3(event):
    overlap_size: int = 3
    x, y = event.x, event.y
    # list of object ids in box
    points: tuple = canvas.find_overlapping(x-overlap_size, y-overlap_size, x+overlap_size, y+overlap_size)
    # Clicked a point
    if len(points) > 0 and points[-1] != 1:
        # id of point to remove
        point_id = points[-1]
        # get road_segment id
        seg_id, lane_id = get_point_data(point_id)
        # remove from map model
        map_model.road_segments[seg_id].remove_point(lane_id, point_id)

        # remove from canvas
        canvas.delete(point_id)
        rerender_road_segments_panel()


# Get point info from the tag data
def get_point_data(point_id) -> tuple:
    tags: List[str] = canvas.gettags(point_id)
    seg_id_str = tags[0]
    lane_id_str = tags[1]
    seg_id = int(seg_id_str[1:])
    lane_id = int(lane_id_str[1:])
    return seg_id, lane_id


# Size is radius, returns id of point
def draw_point(x, y, tags: Union[str, Tuple], size: int = 3) -> int:
    # include tag of segment_id
    return canvas.create_oval(x-size, y-size, x+size, y+size, fill='red', outline='yellow', tags=tags)


# clear existing highlights
def clear_point_highlights():
    for pid in canvas.find_all():
        # Ignore the image on the canvas
        if pid != 1:
            canvas.itemconfigure(pid, outline='')


# Highlight the points that belong to the given segment id
def highlight_seg_points(segment_id: int):
    clear_point_highlights()

    points = canvas.find_withtag(f's{segment_id}')
    # highlight points with matching tag
    for index, pid in enumerate(points):
        if index == 0:
            canvas.itemconfigure(pid, outline='blue')
        else:
            canvas.itemconfigure(pid, outline='yellow')


# Highlight points that belong to the specified path
def highlight_path(path_id=selected_path_id):
    clear_point_highlights()
    sel_path: Path = map_model.paths[path_id]
    # print(f'{sel_path}')
    # Empty path
    if sel_path.empty():
        pass

    for con, p in sel_path.connections:
        canvas.itemconfigure(p.pid, outline='yellow')


# Some wrapper classes for basic GUI elements
class RoadSegmentLabel(tk.Label):
    def __init__(self, segment_id: int, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.segment_id = segment_id


class LaneLabel(tk.Label):
    def __init__(self, lane_id: int, segment_id: int, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.lane_id = lane_id
        self.segment_id = segment_id


class PathLabel(tk.Label):
    def __init__(self, path_id: int, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.path_id = path_id


class RoadSegmentFrame(tk.Frame):
    def __init__(self, segment_id: int, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.segment_id = segment_id


def create_road(seg_type: RoadSegmentType):
    map_model.add_road_segment(seg_type)
    rerender_road_segments_panel()


# Create a new lane in the selected RoadSegment
def create_lane():
    if selected_seg_id == -1:
        pass
    map_model.road_segments[selected_seg_id].add_lane()
    rerender_road_segments_panel()


def create_path():
    map_model.add_path()
    rerender_paths_panel()


def rerender_road_segments_panel():
    clear_frame(seg_frame)
    # Render each RoadSegment in the map
    for key, value in map_model.road_segments.items():
        # different style for selected segment
        relief = 'sunken' if selected_seg_id == key else 'flat'
        bg = '#D3D3D3' if selected_seg_id == key else '#fff'
        # create a label for each road segment
        fr = RoadSegmentFrame(segment_id=value.segment_id, master=seg_frame)
        # Title
        label = RoadSegmentLabel(segment_id=value.segment_id, master=fr, bg=bg,
                                 text=value.seg_type, anchor='w', justify='left', relief=relief)
        label.bind('<Button-1>', handle_select_segment)
        label.pack(side='top', fill='x')
        # Render each Lane in the RoadSegment
        for index, lane in enumerate(value.lanes):
            lane_label = LaneLabel(lane_id=index, segment_id=value.segment_id, master=fr, text=lane, wraplength=300)
            lane_label.bind('<Button-1>', handle_select_lane)
            lane_label.pack(side='top', fill='x')

        fr.pack(side='top', fill='x')


def rerender_paths_panel():
    clear_frame(path_frame)

    if not map_model.paths:
        label = tk.Label(text='No paths', master=path_frame, wraplength=300)
        label.pack(side='top', fill='x')
        return

    for index, path in enumerate(map_model.paths):
        label = PathLabel(path_id=index, text=path, master=path_frame, wraplength=300)
        label.bind('<Button-1>', handle_select_path)
        label.pack(side='top', fill='x')


# Rerender the points in all road_segments. Called on map load from file.
def rerender_points():
    # Delete all points
    for pid in canvas.find_all():
        if pid != 1:
            canvas.delete(pid)

    # Redraw them on the canvas
    for sid, road_segment in map_model.road_segments.items():
        # Update the point ids in the lanes
        for index, lane in enumerate(road_segment.lanes):
            for p in lane.points:
                # Get and assign new ids
                new_pid = draw_point(p.x, p.y, tags=(f's{sid}', f'l{index}'))
                p.update_pid(new_pid)


# Update the path pids when the map pids change. Call after rerender_points()
def update_path_pids():
    # TODO: implement
    pass


def clear_seg_panel_highlights():
    for fr in seg_frame.winfo_children():
        for label in fr.winfo_children():
            label['relief'] = 'flat'
            label['bg'] = '#fff'


def clear_path_panel_highlights():
    for label in path_frame.winfo_children():
        label['relief'] = 'flat'
        label['bg'] = '#fff'


def handle_select_segment(event):
    global selected_seg_id
    global selected_lane_id
    global selected_path_id

    clicked = event.widget
    clear_seg_panel_highlights()
    clear_path_panel_highlights()

    clicked['relief'] = 'sunken'
    clicked['bg'] = '#D3D3D3'

    selected_seg_id = clicked.segment_id
    selected_lane_id = -1
    selected_path_id = -1
    # Update point highlights
    highlight_seg_points(selected_seg_id)
    set_mode(Mode.POINT)


def handle_select_lane(event):
    global selected_seg_id
    global selected_lane_id
    global selected_path_id
    # Handle the label highlighting
    clicked: LaneLabel = event.widget
    parent: RoadSegmentFrame = clicked.master

    clear_seg_panel_highlights()
    clear_path_panel_highlights()

    clicked['relief'] = 'sunken'
    clicked['bg'] = '#D3D3D3'
    # update selected lane id
    selected_seg_id = parent.segment_id
    selected_lane_id = clicked.lane_id
    selected_path_id = -1

    # Handle the point highlighting
    highlight_lane_points()
    set_mode(Mode.POINT)


def handle_select_path(event):
    global selected_seg_id
    global selected_lane_id
    global selected_path_id

    clicked = event.widget
    clear_seg_panel_highlights()
    clear_path_panel_highlights()

    clicked['relief'] = 'sunken'
    clicked['bg'] = '#D3D3D3'

    selected_seg_id = -1
    selected_lane_id = -1
    selected_path_id = clicked.path_id
    set_mode(Mode.PATH)

    clear_point_highlights()
    highlight_path()


def highlight_lane_points():
    clear_point_highlights()
    points = canvas.find_withtag(f's{selected_seg_id}')
    # Filter for points in the lane
    lane_points = [e for e in points if canvas.gettags(e)[1] == f'l{selected_lane_id}']
    # highlight points with matching tag
    for index, pid in enumerate(lane_points):
        # Highlight
        if index == 0:
            canvas.itemconfigure(pid, outline='blue')
        else:
            canvas.itemconfigure(pid, outline='yellow')

# TODO: delete lane?


# Removes all contents from the frame
def clear_frame(fr):
    for widget in fr.winfo_children():
        widget.destroy()


# Not really used for much yet
def handle_undo(event):
    if selected_path_id != -1:
        map_model.paths[selected_path_id].remove_last_point()
        rerender_paths_panel()
        highlight_path()


def init_menu_bar():
    # Menu bar
    menubar = tk.Menu(window)
    window.config(menu=menubar)
    # Create menu
    create_menu = tk.Menu(menubar, tearoff=False)
    create_menu.add_command(
        label='straight road',
        command=lambda: create_road(RoadSegmentType.STRAIGHT)
    )
    create_menu.add_command(
        label='curved road',
        command=lambda: create_road(RoadSegmentType.TURN)
    )
    create_menu.add_command(
        label='intersection',
        command=lambda: create_road(RoadSegmentType.INTERSECTION)
    )
    create_menu.add_command(
        label='lane',
        command=create_lane
    )
    create_menu.add_command(
        label='path',
        command=create_path
    )
    menubar.add_cascade(
        label='Create',
        menu=create_menu
    )
    # File menu
    file_menu = tk.Menu(menubar, tearoff=False)
    file_menu.add_command(
        label='save map',
        command=lambda: MS.save_to_file(map_model)
    )
    file_menu.add_command(
        label='load map',
        command=handle_load
    )
    menubar.add_cascade(
        label='File',
        menu=file_menu
    )

    # Mode menu
    mode_menu = tk.Menu(menubar, tearoff=False)
    mode_menu.add_command(
        label='point',
        command=lambda: set_mode(Mode.POINT)
    )
    mode_menu.add_command(
        label='link',
        command=lambda: set_mode(Mode.LINK)
    )
    mode_menu.add_command(
        label='path',
        command=lambda: set_mode(Mode.PATH)
    )
    menubar.add_cascade(
        label='Mode',
        menu=mode_menu
    )


# For debug
def print_converted_path(event):
    if selected_path_id == -1:
        pass
    path_points = map_model.convert_path(selected_path_id)

    # Format that print-out like the coords.txt file that "simple_path" can interpret
    for x, y in path_points:
        print(f'{x}:{y}')


# Root window
window = tk.Tk()
window.title("Map creator")
init_menu_bar()

# Canvas
img = ImageTk.PhotoImage(Image.open('NH_Top.png'))
h = img.height()
w = img.width()

# Road segments panel
seg_frame = tk.Frame(master=window)
label_a = tk.Label(master=seg_frame, text='no segments yet')
label_a.pack()

# Paths panel
path_frame = tk.Frame(master=window)
label_b = tk.Label(master=path_frame, text='no paths yet')
label_b.pack()

canvas = tk.Canvas(window, width=w, height=h)
canvas.create_image(0, 0, image=img, anchor='nw')
canvas.pack(side="left", fill="both", expand="yes")
# Canvas event handlers
canvas.bind("<Button-1>", handle_canvas_m1)
canvas.bind("<Button-3>", handle_canvas_m3)
window.bind_all('<Control-z>', handle_undo)
window.bind_all('<KeyPress-p>', print_converted_path)

# No idea why this is the order and not the reverse...
path_frame.pack(side="right", fill="both", expand="yes")
seg_frame.pack(side="right", fill="both", expand="yes")

window.mainloop()
