import numpy as np
import matplotlib.pyplot as plt
import streamlit as st

class Bezier:
    def __init__(self, c):
        self.order = len(c)
        self.c = c
    
    def draw_bezier(self):
        t_values = np.linspace(0, 1, 100)
        bezier_points = [self.get_bezier(t) for t in t_values]
        fig, ax = plt.subplots()
        # 绘制贝塞尔曲线
        ax.plot(*zip(*bezier_points), label="Bezier Curve")
        # 绘制控制点和控制多边形
        # plt.scatter(*zip(*self.c), color='red', label="Control Points")
        # plt.plot(*zip(*self.c), color='red', linestyle='--', label="Control Polygon")
        ax.legend()
        st.pyplot(fig)
        # plt.show()

    def get_bezier(self, t):
        return sum([self.c[i] * ((1-t)**(self.order-i-1)) * (t**i) for i in range(self.order)])
    

if __name__ == "__main__":
    st.title("Bezier Curve")
    c1x = st.slider('point 1 x', 0.0, 1.0, 0.0, step=0.01)
    c1y = st.slider('point 1 y', 0.0, 1.0, 0.0, step=0.01)
    c2x = st.slider('point 2 x', 0.0, 1.0, 0.1, step=0.01)
    c2y = st.slider('point 2 y', 0.0, 1.0, 0.1, step=0.01)
    c3x = st.slider('point 3 x', 0.0, 1.0, 0.5, step=0.01)
    c3y = st.slider('point 3 y', 0.0, 1.0, 0.5, step=0.01)
    c4x = st.slider('point 4 x', 0.0, 1.0, 1.0, step=0.01)
    c4y = st.slider('point 4 y', 0.0, 1.0, 1.0, step=0.01)
    c = np.array([[c1x, c1y], [c2x, c2y], [c3x, c3y], [c4x, c4y]])
    bezier = Bezier(c)
    bezier.draw_bezier()
    pass