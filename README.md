# Projeto Pick and Place - Cinemática direta com UR5 + Robotiq gripper

Nesta atividade você deverá tentar implementar a movimentação de um braço robótico UR5 _SEM_ utilizar o MoveIt.

O projeto será implementado em 2 etapas:

1. Pick and Place de um cubo com o UR5
2. Pick and Place de 03 cubos com o UR5

Para isso você deverá utilizar como ferramentas:

- Script Python: ur5_cinematica_direta.py
- A aplicação rqt_send_joint_trajectory que posiciona as juntas do UR5 diretamente no Gazebo

--- 

## Etapa 1: Pick and place de um cubo

**Objetivo**:
- Pegar o cubo pequeno e posicionar ao centro do quadrado maior.

---

Como iniciar o projeto:

1. Execute a aplicação ur5_projeto_etapa1.launch - O Gazebo com o cenário do projeto e o rqt_send_joint vão iniciar

```bash
roslaunch ur5_gazebo ur5_projeto_etapa1.launch

```

2. Utilize o rqt_send_joint_trajectory para posicionar o braço. Anote os valores dos angulos das juntas.

3. Altere a função principal _main_ da aplicação para definir as posições desejadas do braço robótico:

```python
# Define positions (in radians)
    # Position A: Home position
    pos_a = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]

```
As juntas são representadas no vetor na seguinte ordem:
    ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


4. Para executar o script de cinemática direta:

```bash
rosrun ur5_gazebo ur5_cinematica_direta.py

```
---

## Etapa 2: Pick and Place com 03 cubos

**Objetivo:**
Empilhar os 03 cubos pequenos sobre o quadrado maior o mais próximo possível do centro.

1. Execute a aplicação ur5_projeto_etapa2.launch - O Gazebo com o cenário do projeto e o rqt_send_joint vão iniciar

```bash
roslaunch ur5_gazebo ur5_projeto_etapa2.launch

```

Repita os procedimentos da etapa 01 para posicionar os cubos conforme o  objetivo.

---

### Como executar:
Você pode executar em sua máquina local ou via GitHub Codespaces. Se for executar no seu computador é necessária a preparação do ambiente local, caso contrário vá para a seção ["Tutorial Via GitHub Codespaces"](#tutorial-via-github-codespaces).

## Preparação do ambiente na máquina local - git + docker + vscode

1. Verifique se o git está instalado

    ```bash
    git --version
    ```

    1.1 - Se não estiver instalar:

    ```bash
    sudo apt install git
    ```

2. Baixar o repositorio em sua máquina local:

    ```bash
    git clone https://github.com/NURIA-IFSP/LabVir_moveit_ur10_lab1.git
    ```

3. Abrir o vscode no diretório do projeto:

    ```bash
    code LabVir_moveit_ur10_moveit_lab01
    ```

4. Garanta que o docker esteja instalado e rodando:

    ```bash
        docker --version
    ```

5. Se não tiver, instalar o docker:

    [Docker Installation Guide](https://docs.docker.com/get-started/get-docker/)

6. Instale a extensão remote - developement: workspace, no vscode:

    - No menu de extensões do VsCode Ctrl + Shift + X procure por: Remote - Development: Workspace

## Execução do container no vscode

1. Clique no botão de play no canto inferior esquerdo do vscode:
    ![image](https://user-images.githubusercontent.com/10620355/221400332-30592847-0224-491f-9347-138279a71770.png)

2. Clique em "Reopen in Container"

3. Aguarde o container ser iniciado, o vscode irá reiniciar e abrir novamente. (Isso deve levar alguns minutos)

## Abra o ambiente de desenvolvimento no seu browser

1. Abra o terminal PORTS do vscode com o atalho: Ctrl + Shift + P - Forward a Port

2. Clique na primeira porta que estará mapeada no endereço:  <http://localhost:6080>

3. O ambiente XFCE4 deverá abrir no seu browser

4. Se desejar, ajuste a resolução para o seu monitor clicando no canto superior esquerdo do ambiente XFCE4 e selecionando "Display Settings"

5. Clique no botão para estender a exibição para a tela inteira - atalho: Ctrl + Shift + F12

6. Clique no ícone da área de trabalho - "Init_ROS"

    Isso deverá abrir o terminal e exibir a preparação do ambiente ROS (deve levar alguns minutos.)

7. Abra um terminal "Applications -> Terminal Emulator"

8. Mude para o diretorio /LabVir_moveit_ur10_lab1/catkin_ws


## Tutorial Via GitHub Codespaces

Na execução via github Codespaces você não precisará instalar nada em seu computador, terá apenas que ter uma conta no github.

1. Acesse o repositório do projeto no github:
    [https://github.com/NURIA-IFSP/LabVir_moveit_tutorial](https://github.com/NURIA-IFSP/LabVir_moveit_tutorial)
    - Clique no botão "Code" e selecione "Codespaces"
    - O ambiente começará a ser montado no Codespaces (isso pode levar alguns minutos)

2. Feito isso abra o ambiente de desenvolvimento no seu browser, conforme explicado anteriormente e siga os mesmos passos.

3. Avisos Importantes para simulação usando Codespaces:
    - Após a execução do ambiente você deverá clicar no botão "Stop" para encerrar o ambiente.
    - A execução de ambientes de desenvolvimento é cobrada pelo github, havendo um limite atual de 60 horas de execução por mês, ou 180 horas por mês para usuários com acesso premium. Estudantes e professores podem ter o limite aumentado.
