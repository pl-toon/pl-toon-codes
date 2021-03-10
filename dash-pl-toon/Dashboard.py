"""
|---------------------------------------------------|
|       Dash interactivo, con el usuario            |
|     envia los parametros via mqtt a la esp32      |
|       (falta leer datos en tiempo real)           |
|___________________________________________________|
"""
import dash
import dash_core_components as dcc
import dash_html_components as html
from dash.exceptions import PreventUpdate
from dash.dependencies import Input, Output, State
import plotly
import plotly.graph_objects as go
import plotly.express as px
import pandas as pd
import paho.mqtt.publish as publish
import csv

#puerto mqtt
mqttserver = '192.168.100.12'
file_name = 'C:\\Users\\sapet\\Dropbox\\git\\dash_tren2\\prueba_4.csv'

external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']
app = dash.Dash(__name__, external_stylesheets=external_stylesheets)
#Cuadro de TEXTO
markdown_text = '''
### ________________E-Lab of Things________________ 
Un controlador PID (controlador proporcional, integral y derivativo) 
es un mecanismo de control simultaneo por realimentacion ampliamente 
usado en sistemas de control industrial. Este calcula la desviacion 
o error entre un valor medido y un valor deseado.
'''
#Diccionario de colores
colors = {
    'background': '#000000',
    'letters' : '#C0C0C0',
    'text': '#FF0000',
    'grafica': '#F0F787'
}

fig = {}
app.layout = html.Div(
    style={'backgroundColor': colors['background']}, children=[
    html.H1(
        children='Panel de Control Estaciones Remotas 1.3',
        style={
            'textAlign': 'center',
            'color': colors['text']
        }
    ),
    dcc.Markdown(style={'color': colors['letters']}, children=markdown_text),
    html.Div(' '),
    html.Div(children='Control PID: Carro-tren modificado', style={
        'textAlign': 'center',
        'color': colors['grafica'] 
    }),
    html.Div([
        dcc.Interval(
            id='rerefresh',
            disabled=False,
            n_intervals=0,
            interval=1*500,
            max_intervals=-1,
        ),
        dcc.Graph(
            id='example-graph-2',
            figure=fig
        ),
        html.Div(id='output-state2'),
        html.Div(id='output-state3')
    ]),
    html.Div(style={'columnCount': 2},children=[             
    #KP
        #html.Label(children='Kp',style={'textAlign': 'center','color': colors['letters']}),
        dcc.Slider(
                id='input_kp',
                min=0,
                max=300,
                marks={i: '{}'.format(i) for i in range(0, 300, 10)},
                value=0,
                step=0.01,
            ),
        html.Br(),
    #Ki
        #html.Label(children='Ki',style={'textAlign': 'center','color': colors['letters']}),
        dcc.Slider(
                id='input_ki',
                min=0,
                max=300,
                marks={i: '{}'.format(i) for i in range(0, 300, 10)},
                value=0,
                step=0.01,
            ),
        html.Br(),
    #Kd
        #html.Label(children='Kd',style={'textAlign': 'center','color': colors['letters']}),
        dcc.Slider(
                id='input_kd',
                min=0,
                max=150,
                marks={i: '{}'.format(i) for i in range(0, 150, 10)},
                value=0,
                step=0.01,
            ),
        html.Br(),
        #mitad de la cosa
        html.Label(id='output_kp',style={'textAlign': 'center','color': colors['letters']}),
        html.Br(),
        html.Label(id='output_ki',style={'textAlign': 'center','color': colors['letters']}),
        html.Br(),
        html.Label(id='output_kd',style={'textAlign': 'center','color': colors['letters']}),
        html.Br(),
    ]),
    html.Div(children=[
        html.Label('Referencia (cm)',style={'textAlign': 'center','color': colors['letters']}),
        ]),
    html.Div(style={'textAlign': 'center'},children=[
        dcc.Input(id="input-referencia", type="number", value=15,style={'textAlign': 'center'},min=5, max=100,step=10),
        html.Br()]),
        html.Br(),
    html.Div(children=[
        html.Button(id='init-button', n_clicks=0, children='Start Mesurement'),
        html.Button(id='stop-button', n_clicks=0, children='Stop x-pirience'),
        html.Div(id='output-state')
    ],
     style={'textAlign': 'center','color': colors['letters']})
])

@app.callback(Output(component_id='output-state2',component_property='children'),
            [Input(component_id='init-button', component_property='n_clicks')])
def inicio_prueba(n):
    if n==0:
        raise PreventUpdate
    else:
        publish.single('esp/inicio', n, hostname=mqttserver)
    return

#auto refresh del grafico
@app.callback(Output(component_id='example-graph-2',component_property='figure'),
            [Input(component_id='rerefresh', component_property='n_intervals')])
def actualizar(n):
    if n==0:
        raise PreventUpdate
    else:
        list_id =[]
        list_mensaje = []
        with open(file_name, 'r', newline='') as csv_file:
            csv_reader = csv.reader(csv_file)
            next(csv_reader)
            for line in csv_reader:
                list_id.append(line[0])
                list_mensaje.append(line[1])
        df = pd.DataFrame({"": list_id, "Distancia": list_mensaje}) #datos del grafico, falta interaccion con mysql
        fig = px.line(df, x="", y="Distancia")
        fig.update_layout(plot_bgcolor=colors['background'], paper_bgcolor=colors['background'], font_color=colors['grafica'])
    return(fig)

#Detener o comenzar refresh
@app.callback(Output('rerefresh','max_intervals'),
             [Input('init-button','n_clicks'),
             Input('stop-button','n_clicks')])
def refresh_stopstar(boton2,boton3):
    ctx = dash.callback_context
    if not ctx.triggered:
        max_intervals = -1
        publish.single('esp/ON', '1', hostname=mqttserver)
    else:
        button_id = ctx.triggered[0]['prop_id'].split('.')[0]
        if button_id == 'init-button':
            max_intervals = -1
        if button_id == 'stop-button':
            publish.single('esp/apagar', boton3, hostname=mqttserver)
            max_intervals = 0
    return (max_intervals)

#slider kp
@app.callback(Output(component_id='output_kp', component_property='children'),
             [Input(component_id='input_kp', component_property='value')])
def update_output_div(input_value):
    publish.single('trenes/carroD/p', input_value, hostname=mqttserver)
    return 'Kp: {}'.format(input_value)

#slider ki
@app.callback(Output(component_id='output_ki', component_property='children'),
            [Input(component_id='input_ki', component_property='value')])
def update_output_div(input_value):
    publish.single('trenes/carroD/i', input_value, hostname=mqttserver)
    return 'Ki: {}'.format(input_value)

#slider kp
@app.callback(Output(component_id='output_kd', component_property='children'),
            [Input(component_id='input_kd', component_property='value')])
def update_output_div(input_value):
    publish.single('trenes/carroD/d', input_value, hostname=mqttserver)
    return 'Kd: {}'.format(input_value)

#number box
@app.callback(Output(component_id='output-state3', component_property='children'),
            [Input(component_id='input-referencia', component_property='value')])
def update_output_div(input_value):
    publish.single('trenes/ref', input_value, hostname=mqttserver)
    return

if __name__ == '__main__':
    app.run_server(debug=True)