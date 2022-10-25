from math import floor
from PIL import Image
import numpy as np

def _x(i:int, xres:int) -> int:
    if (i < 0): i = 0
    return i % xres

def _y(i:int, xres:int) -> int:
    if (i < 0): i = 0
    return floor(i / xres)

def _i(x:int, y:int, xres, yres) -> int:
    if (x < 0): x = 0
    if (y < 0): y = 0
    if (x >= xres): x = xres - 1
    if (y >= yres): y = yres - 1
    return y * xres + x
    
def _clamp(n, smallest, largest):
    return max(smallest, min(n, largest))


def getneighbors(arr:np.ndarray, x, y):
    """
    Get the 3x3 neighborhood around (x,y) in "arr"

    Follows the PIL convention when viewed face-on
    ```
                #(y)
            #ul #uc #ur 
    #(x)    #ll #cc  #rr
            #dl #dc #dr
    ```
    """
    
    x_dim = arr.shape[0]
    y_dim = arr.shape[1]
    if len(arr.shape) == 3:
        z_dim = arr.shape[2]
    else:
        z_dim = 0

    assert(x < x_dim and y < y_dim)

    out = np.zeros((3,3, z_dim), dtype=arr.dtype)
    
    _u = _clamp(x - 1, 0, x_dim - 1)
    _d = _clamp(x + 1, 0, x_dim - 1)
    _l = _clamp(y - 1, 0, y_dim - 1)
    _r = _clamp(y + 1, 0, y_dim - 1)

    out[0][:] = [arr[_u][_l], arr[_u][y], arr[_u][_r]]
    out[1][:] = [arr[x][_l], arr[x][y], arr[x][_r]]
    out[2][:] = [arr[_d][_l], arr[_d][y], arr[_d][_r]]
    
    return out


def bayer_bggr_to_rgb_bilinear(img):
    # img = Image.open(srcimg)
    bayer = np.asarray(img, dtype=np.uint8)
    x_dim = bayer.shape[0]
    y_dim = bayer.shape[1]
    z_dim = bayer.shape[2]
    out = np.zeros((x_dim, y_dim, z_dim), dtype=np.uint8)

    for x in range(x_dim):
        for y in range(y_dim):
            neighbors = getneighbors(bayer, x, y)
            if not x & 0b1: # Even row B G B G B G
                if not y & 0b1: # B
                    r = sum([
                        neighbors[0][0][0], # Top left
                        neighbors[0][2][0], # Top right
                        neighbors[2][0][0], # Bottom left
                        neighbors[2][2][0]  # Bottom right
                    ]) / 4
                    g = sum([
                        neighbors[0][1][1], # Up
                        neighbors[2][1][1], # Down
                        neighbors[1][0][1], # Left
                        neighbors[1][2][1] # Right
                    ]) / 4
                    b = bayer[x][y][2]
                else: # G
                    r = sum([
                        neighbors[0][1][0], # Up
                        neighbors[2][1][0]  # Down
                    ]) / 2
                    g = bayer[x][y][1]
                    b = sum([
                        neighbors[1][0][2], # Left
                        neighbors[1][2][2]  # Right
                    ]) / 2
            else: # Odd row G R G R G R
                if not y & 0b1: # G
                    r = sum([
                        neighbors[1][0][0], # Left
                        neighbors[1][2][0]  # Right
                    ]) / 2
                    g = bayer[x][y][1]
                    b = sum([
                        neighbors[0][1][2], # Up
                        neighbors[2][1][2]  # Down
                    ]) / 2
                else: # R
                    r = bayer[x][y][0]
                    g = sum([
                        neighbors[0][1][1], # Up
                        neighbors[2][1][1], # Down
                        neighbors[1][0][1], # Left
                        neighbors[1][2][1] # Right
                    ]) / 4
                    b = sum([
                        neighbors[0][0][2], # Top left
                        neighbors[0][2][2], # Top right
                        neighbors[2][0][2], # Bottom left
                        neighbors[2][2][2]  # Bottom right
                    ]) / 4
            
            out[x][y][:] = [_clamp(r,0,255), _clamp(g,0,255), _clamp(b,0,255)]
                    
    return Image.fromarray(out)

if __name__ == "__main__":
    img = Image.open("test/testpattern.png")
    test = bayer_bggr_to_rgb_bilinear(img)
    test.save("test/testpattern-bilinear.png")
