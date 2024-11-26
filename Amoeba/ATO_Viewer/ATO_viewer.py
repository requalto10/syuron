import re
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib as mpl
#from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox, Slider, Button
import numpy as np
from typing import Optional, NoReturn
import ast
from tqdm import tqdm
import colorsys
import argparse
import pathlib
import json
import sys
import time
position_map_default = {1: (0, 5), 2: (1, 5), 3: (2, 5), 4: (3, 5), 5: (4, 5),
                6: (5, 5), 7: (0, 4), 8: (1, 4), 9: (2, 4), 10: (3, 4),
                11: (4, 4), 12: (5, 4), 13: (0, 3), 14: (1, 3), 15: (2, 3),
                16: (3, 3), 17: (4, 3), 18: (5, 3), 19: (0, 2),
                20: (1, 2), 21: (2, 2), 22: (3, 2), 23: (4, 2), 24: (5, 2),
                25: (0, 1), 26: (1, 1), 27: (2, 1), 28: (3, 1),
                29: (4, 1), 30: (5, 1), 31: (0, 0), 32: (1, 0), 33: (2, 0),
                34: (3, 0), 35: (4, 0), 36: (5, 0)}


class Draw_ATO:
    """
    ATOのデータを描画するクラス
   """

    def __init__(
            self,
            pos_map: dict[int, tuple[int, int]],
            dat: list,
            plate_dat: list,
            ranges: Optional[list[tuple[int, int]]] = None,
            view: list[int] = [],
            cars: Optional[list[int] | int] = None,
            zscale: int = 1,
            plate_size: int = 5
    ):
        """
        pos_map: cellの番号と2次元位置の対応Dictonary
        dat:     結果データ
        ranges:  x,y,z軸の表示範囲
        """
        self.ranges = ranges
        self.position_map = pos_map
        self.fig = plt.figure(figsize=(8.0, 6.0))
        self.ax_list = []
        self.QUAD = np.array([
            [0, 0, 0],
            [1, 0, 0],
            [1, 1, 0],
            [0, 1, 0],
        ])
        self.dat = dat
        self.plate_dat = plate_dat
        self.plate_size = plate_size
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.num_of_cars = len(self.dat[0])
        self.data_size = len(self.dat)

        self.color_map = []
        # define a colormap for each vehicle
        if cars is None:
            cars = list(range(self.num_of_cars))
        elif type(cars) == 'int':
            cars = [cars]
        self.cars = cars
        for vehicle_idx in range(self.num_of_cars):
            hue = float(vehicle_idx) / float(self.num_of_cars)
            # translate HSV to RGB for matplotlib
            edge_color = colorsys.hsv_to_rgb(hue, 0.2, 0.2)
            face_color = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
            self.color_map.append(
                {
                    'edge_color': edge_color,
                    'face_color': face_color,
                }
            )
            self.bar = None
        if view is None or len(view) < 2:

            raise Exception('two integers needed for the option, view ')
        self.view = view
        self.zscale = zscale
    def make_polygons_for_one_vehicle(self,
                                      frame: int,
                                      vehicle_idx: int
                                      ) -> list[np.ndarray]:
        """
        台車1台分のpolygonを作成
        frame:       frame (step)番号
        vehicle_idx  台車の番号
        return:      台車1台分のpolygon
        """
        polygons = []
        for pos in self.dat[frame][vehicle_idx]:
            self.make_polygons_for_cell(pos, polygons)
        return polygons

    def draw_one_step(self, frame: int) -> NoReturn:
        """
        ある frame (step) の描画。matplotlibのFuncAnimationに渡す
        update関数として機能
        frame: frame (step) 番号。FuncAnimationから渡される
        """
        # clear axes
        self.ax.cla()

        # set timestamp as a title string
        self.ax.set_title(f"t={frame}")
        mpl.rcParams['axes.autolimit_mode'] = 'round_numbers'
        mpl.rcParams['axes.zmargin'] = 0
        # set graph ragages and aspect ratios
        self.ax.axes.set_xlim3d(self.ranges[0][0], self.ranges[0][1]+1, view_margin=0)
        self.ax.axes.set_ylim3d(self.ranges[1][0], self.ranges[1][1]+1, view_margin=0)
        #self.ax.axes.set_zlim3d(self.ranges[2][0]+1, self.ranges[2][1])
        self.ax.axes.set_zlim3d(self.ranges[2][0], self.ranges[2][1]+1, view_margin=0)

        self.ax.set_box_aspect((
            (self.ranges[0][1] - self.ranges[0][0])*2,
            (self.ranges[1][1] - self.ranges[1][0])*2,
            (self.ranges[2][1] - self.ranges[2][0])*self.zscale
        ))
        # remove tick numbers
        self.ax.xaxis.set_ticklabels([])
        self.ax.yaxis.set_ticklabels([])
        self.ax.xaxis.set_ticks(np.arange(self.ranges[0][0], self.ranges[0][1]+1))
        self.ax.yaxis.set_ticks(np.arange(self.ranges[1][0], self.ranges[1][1]+1))
        self.ax.invert_yaxis()
        if self.view is not None and len(self.view) == 2:
            self.ax.view_init(elev=self.view[0], azim=self.view[1])
        # draw all
        for vehicle_idx in self.cars:
            car = self.make_polygons_for_one_vehicle(frame, vehicle_idx)
            self.ax.add_collection3d(
                Poly3DCollection(
                    car,
                    facecolors=self.color_map[vehicle_idx]['face_color'],
                    edgecolors=self.color_map[vehicle_idx]['edge_color'],
                    alpha=0.2,
                    linewidth=0.25,
                )
            )
        for plate_v in self.plate_dat[frame]:
            for p, plate_pos in enumerate(plate_v):
                self.draw_one_plate(p, plate_pos)

        if self.bar is not None:
            self.bar.update(1)

    def draw_one_plate(self, p:int, pos:list):
        polygon = [
            [pos[1]-self.plate_size//2,   pos[0]-self.plate_size//2,   p],
            [pos[1]+self.plate_size//2+1, pos[0]-self.plate_size//2,   p],
            [pos[1]+self.plate_size//2+1, pos[0]+self.plate_size//2+1, p],
            [pos[1]-self.plate_size//2, pos[0]+self.plate_size//2+1, p],
        ]
        self.ax.add_collection3d(
                Poly3DCollection(
                    [polygon],
                    edgecolors='black',
                    facecolors='white',
                    alpha=0,
                    linewidth=0.2
                )
        )
        
    def make_polygons_for_cell(
            self, pos: list[int],
            polygons: list
    ) -> NoReturn:
        """
        cell一つ分のpolygonを生成
        pos: cell一つを表現するリスト
          pos[0]: timestep d
          pos[1]: deature
          pos[2]: destination
          pos[3]: hight of the cell
        polygons: list of polygons (戻り値用）
        """
        pb = pos[0]
        pt = pb + pos[3]
        # make top and bottom polygons
        try:
            posb = self.position_map[pos[1]]
            post = self.position_map[pos[2]]
        except KeyError:
            return
        #if np.abs(posb[0] - post[0]) > 1.1 or np.abs(posb[1] - post[1]) > 1.1:
        #    return
        bottom_poly = self.QUAD + np.array([posb[0], posb[1], pb])
        top_poly = self.QUAD + np.array([post[0], post[1], pt])
        polygons.append(bottom_poly)
        polygons.append(top_poly)

        # make four side polygons
        for idx_i in range(4):
            idx_j = (idx_i + 1) % 4
            poly = np.array([
                bottom_poly[idx_i],
                bottom_poly[idx_j],
                top_poly[idx_j],
                top_poly[idx_i],
            ])
            polygons.append(poly)

    def animation(self,
                  interval: int = 100,
                  savefile: Optional[str] = None
                  ) -> NoReturn:
        """
        アニメーションを作成
        matplotlibのFuncAnimationを呼び出してアニメーションを作成させる
        interval: アニメーションのインターバル
        savefile: 出力画像ファイル。None(deafult)の場合は画面に表示される
        """

        self.bar = tqdm(total=len(self.dat))
        self.bar.set_description(desc="[movie creation]")
        anim = FuncAnimation(self.fig,
                             self.draw_one_step,
                             frames=len(self.dat),
                             repeat=False,
                             interval=interval)
        if savefile is None:
            plt.show()
        else:
            anim.save(savefile, writer="ffmpeg")

    def _submit_for_text_box(self, expression: str):
        self.view = [self.ax.elev, self.ax.azim]
        try:
            self.step = int(expression)
        except:
            pass
        self.text_box.set_val(str(self.step))
        if self.step >= self.data_size:
            self.step= self.data_size -1 
        self.draw_one_step(self.step)
        self.text_box.set_val(self.step)
        self.slider.set_val(self.step)
        self.fig.canvas.draw_idle()
    def _update_for_slider(self, val):
        self.view = [self.ax.elev, self.ax.azim]
        self.draw_one_step(int(val))
        self.text_box.set_val(int(val))
        self.step = int(val)
        self.fig.canvas.draw_idle()

    def _run_button(self, event):
        self.run = True
        self.run_button.set_active(False)
        self.stop_button.set_active(True)
        self.timer.start()

    def _stop_button(self, event):
        self.run = False
        self.timer.stop()
        self.run_button.set_active(True)
        self.stop_button.set_active(False)
    

    def _timer_callback(self):
        if self.step < self.data_size-1:
            self.step += 1
            self.view = [self.ax.elev, self.ax.azim]
            self.draw_one_step(self.step)
            self.text_box.set_val(str(self.step))
            self.slider.set_val(self.step)
        else:
            self.run = False
            self.timer.stop()
            self.run_button.set_active(True)
            self.stop_button.set_active(False)
        
    def interractive_draw(self, step):
        self.run = False
        self.timer = self.fig.canvas.new_timer(interval=200)
        self.timer.add_callback(self._timer_callback)
        self.step=step
        self.draw_one_step(step)
        axbox =    self.fig.add_axes([0.1,  0.08, 0.8, 0.05])
        axslider = self.fig.add_axes([0.1,  0.02, 0.8, 0.070])
        self.text_box = TextBox(axbox, "step:",initial=str(step), textalignment="center")
        self.slider = Slider(ax=axslider,
                             label='',
                             valmin=0,
                             valmax=self.data_size-1,
                             valinit=self.step,
                             valstep=1,
                             orientation="horizontal")
        self.slider.on_changed(self._update_for_slider)
        self.text_box.on_submit(self._submit_for_text_box)
        ax_runbutton = self.fig.add_axes([0.1, 0.9, 0.1, 0.05])
        self.run_button = Button(
            ax=ax_runbutton,
            label='Run',
        )
        ax_stopbutton = self.fig.add_axes([0.22, 0.9, 0.1, 0.05])
        self.run_button.on_clicked(self._run_button)
        self.stop_button = Button(
            ax=ax_stopbutton,
            label='Stop'
        )
        self.stop_button.on_clicked(self._stop_button)
        self.stop_button.set_active(False)
        time.sleep(0.5)
        plt.show()


def ato_data_loader(data_file: str) -> list:
    """
    結果データのテキストを読み込み、配列データにする関数
    datafile: 結果データ
    return:   結果データをリストにしたもの
    """

    ret = []
    pm = None
    with open(data_file, 'r', encoding='utf-8') as f:
        def parse_header(lines: list[str]):
            pmax = None
            for line in lines:
                if line.split()[0] == 'vmax/pmax':
                    pmax = int(line.split()[2])
                    break
            if pmax is None:
                json_str = " ".join(lines)
                pmax = int(json.loads(json_str)['pmax'])
            return pmax

        header = []
        body = []
        state = 'i'
        for line in f:
            line = line.strip()
            if state == 'i' and line == '---':
                state = 'h'
            elif state == 'h':
                if line == '---':
                    state = 'b'
                else:
                    header.append(line)
            elif state == 'b':
                body.append(line)
        pm = parse_header(header)
        ret = []
        for line in body:
            if len(line) > 0 and line[0] == '[':
                arr = ast.literal_eval(line)
                ret.append(arr)
    return ret, pm


def make_edges(cell_dat: list,
               pos_map: dict[int, tuple[int, int]])->list:
    edge_types = (
        (1, 1, 1, 1), # xx 
        (1, 0, 3, 1), # xz
        (0, 1, 3, 1), # zx
        (0, 0, 5, 1), # zz
        (0, 0, 6, 0), # turn
        (0, 0, 1, 0), # st
    )
    results = []
    inversed_map = {val:key for key,val in pos_map.items()}

    def find_edges(cell, all_cells):
        results = []
        for e in edge_types:
            cond = \
                (cell[1]==e[0]) & \
                (all_cells[:,1]==e[1]) & \
                (cell[0]+e[2]==all_cells[:,0])  & \
                (np.abs(cell[2]-all_cells[:,2])+ np.abs(cell[3]-all_cells[:,3]) == e[3])
            dst = all_cells[np.where(cond)]
            edges = [[cell[0], inversed_map[(cell[2],cell[3])], inversed_map[(d[2],d[3])], e[2]] for d in dst]
            results += edges
        return results


    def trans(state):
        if state == 'x':
            return 1
        else:
            return 0
    
    for cell_t in cell_dat:
        results.append([])
        for cell_v in cell_t:
            cells = np.array([[c[0], trans(c[3])] + list(pos_map[c[1]]) for c in cell_v])
            results[-1].append([])
            for cell in cells:
                results[-1][-1] += find_edges(cell, cells)
    return results
        



def get_ranges(pos_map: dict[int, tuple[int, int]]
              ) -> tuple[int, int, int, int]:
    xvals = [val[0] for val in pos_map.values()]
    yvals = [val[1] for val in pos_map.values()]
    
    return [min(xvals), max(xvals), min(yvals), max(yvals)]


def parse_args() -> argparse.Namespace:
    """
    引数パース用関数
    """
    parser = argparse.ArgumentParser(
        description='make a movie from ATO result data')
    parser.add_argument('data_file_name',
                        type=str,
                        help='set simulated data')
    parser.add_argument(
        '--cell-base',
        action='store_true',
        help='switch edge-base to cell-base'
    )
    parser.add_argument('--plate-data',
                        type=str,
                        default=None,
                        help='set data for positions of plates'
                        )
    parser.add_argument('--plate-size',
                        type=int,
                        default=5,
                        help='set plate size (default=5)'
                        )
    parser.add_argument('--output',
                        type=str,
                        default=None,
                        help='set output file name')
    parser.add_argument('--step',
                        type=int,
                        default=-1,
                        help='visualized step'

                        )
    parser.add_argument('--view',
                        nargs="*",
                        type=int,
                        default=None,
                        help='set view point'
                        )
    parser.add_argument('--position-map',
                        type=str,
                        default=None,
                        help="position map file"
                        )
    parser.add_argument('--ranges',
                        nargs="*",
                        type=int,
                        help='view range',
                        default=None
                        )
    parser.add_argument('--cars',
                        type=int,
                        nargs="*",
                        help='indice of cars',
                        default=None
                        )
    parser.add_argument('--zscale',
                        type=int,
                        help="zscale",
                        default=1
                        )
    return parser.parse_args()


    

DEBUG = False
if __name__ == '__main__' and DEBUG:
    dat, _ = ato_data_loader('evolution_U_grid_36_4D_v4_01_1.dat')
    results = make_edges(dat, position_map_default)
    sys.exit(0)
if __name__ == '__main__':
    args = parse_args()

    # define dictionary from integer indice to 2d positions
    if args.position_map is not None:
        with open(args.position_map) as f:
            position_map = json.load(f)
        position_map = {int("".join(re.findall(r'\d+', key))): tuple(val)
                        for key, val in position_map.items()}
    else:
        position_map = position_map_default

    print("load data...", end="", flush=True)
    dat, pmax = ato_data_loader(args.data_file_name)
    
    print(" done!!", flush=True)
    print("",flush=True)
    if args.cell_base:
        data = make_edges(dat, position_map)
    else:
        data = dat
    if args.plate_data is not None:
        plate_data, _ = ato_data_loader(args.plate_data)
    else:
        plate_data = None

    if args.view is None:
        view = [30, 30]
    else:
        view = args.view

    if args.ranges is None:
        rvals = get_ranges(position_map)
        if rvals[0] == rvals[1]:
            rvals[1] += 1
        if rvals[2] == rvals[3]:
            rvals[3] += 1
        ranges = [
            [rvals[0], rvals[1]],
            [rvals[2], rvals[3]],
            [0, pmax]
        ]
    else:
        ranges = [
            [args.ranges[0], args.ranges[1]],
            [args.ranges[2], args.ranges[3]],
            [0, pmax]
        ]
    draw_ato = Draw_ATO(position_map,
                        plate_dat=plate_data,
                        ranges=ranges,
                        dat=data,
                        view=view,
                        cars=args.cars,
                        zscale=args.zscale,
                        plate_size=args.plate_size
                        )

    if args.step != -1:
        draw_ato.interractive_draw(args.step)
    else:
        if args.output is None:
            # The stem of input data file name + ".mp4" is set as
            # the movie file name if the argument "--output" is not given
            data_file = pathlib.Path(args.data_file_name)
            output = data_file.parent / (data_file.stem + '.mp4')
        else:
            output = pathlib.Path(args.output)
            # start animation
        draw_ato.animation(savefile=output)
