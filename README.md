# Software-Mapeamento

Repositório de Desenvolvimento dos Códigos de Software voltados ao mapeamento e algoritmos utilizados no SLAM (simultaneous location and mapping) e PPCR (path planning of coverage region).
## Sumário

* [Dependências](#Dependências)
* [Manual de Instalação](#Manual-de-Instalação)
* [Estrutura do Projeto](#Estrutura-de-Projeto)
* [Guia de Contribuição](#Guia-de-Contribuição)
    * [Issues](#Issues)
    * [Branches](#Branches)
    * [Commits](#Commits)
    * [Pull Requests](#Pull-Requests)


## [**Dependências**](#Sumário)

Para a execução local da Wiki do projeto serão necessárias as seguintes dependências:

* [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
* [ROS - Pacote Gmapping]()
* [Python 3](https://www.python.org/downloads/)

## [**Manual de Instalação**](#Sumário)

### Instalação de Dependências

Para começar a desenvolver o projeto é necessário, antes de mais nada, instalar as dependências do ROS Noetic e do Python. 

#### **Python**

Para instalar o Python é necesário baixar os pacotes de acordo com o presente na [documentação oficial](https://wiki.python.org/moin/BeginnersGuide/Download) da linguagem. Após a instalação correta, pode-se verificar a versão e se a instalação foi realizada corretamente utilizando o seguinte comando:

 ```bash
  # Executa-se o seguinte comando, esperando o resultado a seguir
  $ python3 --version
 ```

 Após a execução do comando anterior, espera-se o seguinte resultado, com possíveis alterações no número da versão a depender da que foi instalada:

 ```bash
   Python 3.10.0
 ```

#### **ROS Noetic**

Para instalar o ROS Noetic, recomenda-se uma distribuição Linux com poucas ou nenhuma modificação adicional realizada. Para isso, deve-se seguir a [documentação oficial](http://wiki.ros.org/noetic/Installation/Ubuntu). Na data de elaboração desse manual, os seguintes comandos compunham a instalção referenciada:

```bash
# Prepara o sistema para aceitar pacotes da organização packages.ros.org
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Instalar o programa curl
$ sudo apt install curl 

# Setar as Keys paa instalação
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Atualizar a versão do dpkg do sistem
$ sudo apt update

# Instalar a versão Full do ROS Noetic, contendo ferramentas de visualização
$ sudo apt install ros-noetic-desktop-full

# Exportar o script de uso do ROS (Necessário em TODOS os terminais de uso)
# O comando pode conter o último parâmetro como 'setup.bash'
# a depender da versão do terminal
$ source /opt/ros/noetic/setup.bash

# Dependências e pacotes adicionais
$ sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Instalar o rosdep para manejo de dependências
$ sudo apt install python3-rosdep

# Para inicializar o rosdep
$ sudo rosdep init

# Para verificar a versão e se a instalação foi bem sucedida
$ rosdep --version
``` 

#### **ROS - Pacote de Gmapping**

Para instalar o pacote de visualização das estruturas e dados de mapeamento via ROS, é necessário uma instalação de pacotes adicionais após a instalação dos pacotes do ROS Noetic Full instalados anteriormente, sendo necessário executar:

```bash
  # Instalação do pacote de gmapping
  $ sudo apt-get install ros-noetic-gmapping
```

### Execução dos Ambientes

Para executar os ambientes no projeto em questão, é necessário realizar a seguinte configuração.
#### Passo 1: Clone do Repositório
```bash
  # Criar uma pasta para armazenamento do projeto
  $ mkdir Projeto-Cortador-Embarcado

  # Entrar na pasta do projeto
  $ cd Projeto-Cortador-Embarcado/

  # Realizar o clone do repositório
  $ git clone https://github.com/Cortador-de-Grama-Autonomo/Software-Mapeamento
```

Após o projeto estar devidamente adicionado, é necessário executar os comandos para configurar o ambiente onde o desenvolvimento e os testes ocorrerão:

#### Passo 2: Compilar os workspaces

```bash
  # Entrar no repositório do projeto
  $ cd ./Projeto-Cortador-Embarcado/Software-Mapeamento

  # Executar o comando para habilitar o uso de funções do ROS
  # Podendo ser 'setup.bash' a depender da configuração do terminal
  $ source /opt/ros/noetic/setup.zsh

  # Entrar na pasta 'catkin_ws'
  $ cd ./catkin_ws

  # Executar o comando de make
  $ catkin_make

  # Entrar na pasta 'simulation_ws'
  $ cd ../simulation_ws

  # Executar o comando de make do projeto
  $ catkin_make
```

Os arquivos criados pelos comandos anteriores serão

**Catkin_ws**
- `catkin_ws/build/`
- `catkin_ws/devel/`

**Simulation_ws**
- `smilation_ws/build/`
- `smilation_ws/devel/`

### Execução

As seguintes etapas e passos podem ser tomadas para realizar o teste da execução do ambiente, com as dependências de visualizações de dados do ROS, como o [Gazebo](http://gazebosim.org/) e o [Rviz](http://wiki.ros.org/rviz)

```bash
# Executar o ambiente de simulação do Gazebo
 $ cd simulation_ws
 $ source devel/setup.zsh
 $ roslaunch worlds world.launch

# Executar o carro cortador no ambiente do Gazebo
 $ cd simulation_ws
 $ source devel/setup.zsh
 $ roslaunch cortador_description spawn.launch

 # Executar o ambinte de visualização do Rviz
 $ cd catkin_ws
 $ source devel/setup.zsh
 $ roslaunch controle_locomocao gmapping.launch

# Executar o algoritmo necessário
 $ cd catkin_ws
 $ source devel/setup.zsh
 $ rosrun controle_locomocao wall_follow.py

# Para identificar os tópicos do ROS
 $ rostopic list
```
## [**Estrutura do Projeto**](#Sumário)

O Presente projeto contém a seguinte lista de pacotes e diretórios, organizados de forma a contemplar as etapas de desenvolvimento e implementação.
### **Pacotes:**
* **Controle_Locomocao:** Implementação do código de controle e dos nós de comunicação para movimento do robô cortador de grama.
* **Controle_Corte:** Implementação do código de controle da altura e velocidade do corte da grama.
* **Sensores:** Biblioteca de códigos para obtenção de dados dos sensores.
* **worlds:** Ambientes para simulação
* **cortador_description:** Descrição do cortador para simulação

## [**Guia de Contribuição**](#Sumário)

Para a contribuição e evolução das informações presentes nesse repositório, o seguinte guia de contribuição foi criado, especificando instruções de uso e dos padrões utilizados.

### [**Issues**](#Sumário)

As issues devem ser criadas conforme template predefinido, contendo:

* ***Descrição*** - Descrição simples e direta do problema que a issue busca resolver ou da adição da issue;

* ***Tarefas*** - *Checklist* trazendo o passo a passo granularizado de como a issue deve ser executada, permitindo que cada tarefa seja marcada quando concluída;

* ***Critérios de Aceitação*** - *Checklist* do funcionamento ou resultado esperado após a conclusão da issue de forma satisfatória, permitindo que cada critério seja marcado quando concluído;

* ***Issue Vinculada*** - Referência a outra issue que esteja vinculada à issue presente, com link para a issue referenciada. Caso não haja issue, deve ser preenchido como 'Não se aplica.' somente;

* ***Assignees*** - As issues criadas devem ser atribuídas a um membro para execução, se houver alguém responsável por realizá-la;

* ***Labels*** - As issues criadas devem conter labels que categorizem aquela issue, para informar aos demais contribuidores sobre a natureza de seu conteúdo;

* ***Sprint*** - As issues devem possuir a sprint a qual se referem no campo destinado;

* ***Estimate*** - As issues devem ser pontuadas conforme seu grau de dificuldade;

#### **Nomenclatura**

A Nomenclatura das issues deve seguir o padrão:

```
[PREFIXO] Breve descrição da issue em português
```

De forma que o elemento **PREFIXO** siga o seguinte padrão

| Prefixo | Tema | Exemplo |
| --- | --- | --- |
| DOCS | Documentação | `[DOCS] Melhorar README`|
| DEVOPS | Integração, DevOps | `[DEVOPS] Implementar CI/CD`|
| USXX | História de Usuário, em que *XX* se refera ao número da história de usuário | `[US12] Enviar dados de mapeamento para gateway`|
| IMPROVE | Melhoria ou adição de funcionalidades não correlacionadas a uma História de Usuário | `[IMPROVE] Adiciona regras de mapeamento no frontend`|



### [**Branches**](#Sumário)

Para a padronização das branches foi tomada uma adaptação do modelo padrão do [gitflow](https://nvie.com/posts/a-successful-git-branching-model/) conforme representado pelo diagrama abaixo:

![gitflow-adapted](./images/gitflow_adapted_tag.png)

Esse modelo segue a seguinte categorização para as respectivas branches:

* ***Main*** - A Branch *Main* contém o histórico oficial do código ou projeto em questão, sendo assim a versão do código que estará em produção no ciclo de vida do projeto.

* ***Hotfix*** - As Branches *Hotfix* servem para manutenção ou correção de forma rápida dos lançamentos em produção, feitas a partir da *Main* para uma integração mais rápida.

* ***Develop*** - A Branch *Develop* serve como uma ramificação para integração de recursos, sendo a versão do projeto disponível no ambiente de Homologação.

* ***Feature*** - As Branches de *Feature* servem para a realização de novas adições e funcionalidades para o projeto, sendo criadas a partir da branch *develop* para serem integradas a essa branch à medida que a *feature* (ou funcionalidade) em questão é concluída. Essas funcionalidades representam novas adições ao projeto em si, através da inserção de códigos.

* ***Support*** - As Branches *Support* servem para armazenar modificações que integram o projeto mas não representam funcionalidades (ou *features*), como documentação, Integração Contínua, Deploy Contínuo e demais configurações focadas na estrutura do projeto.

#### **Nomenclatura**

A Nomenclatura das branches deve seguir o seguinte padrão
 
| Branch | Nomenclatura |
| --- | --- |
| Main | main |
| Develop | develop |
| Feature | feature/<span style="color:#fc6a03">[NUMERO-USER-STORY]</span>-<span style="color:#ed820e">[BREVE-DESCRIÇÃO-EM-INGLES]</span> <br> Ex.: `feature/01-mark-point-on-map` |
| Support | support/<span style="color:#ffcd01">[BREVE-DESCRIÇÃO-EM-INGLES]</span> <br> Ex.: `support/document-contribution-guide` |
| Hotfix | hotfix/<span style="color:#cf513d">[BREVE-DESCRIÇÃO-EM-INGLES]</span> <br> Ex.: `hotfix/remove-second-callback`  |

### [**Commits**](#Sumário)

Os commits devem seguir padrões paa identificação de alterações feitas e implementadas, tanto para futuras modificações quanto para gerar rastro de informação.

#### **Nomenclatura**

A Padronização de commits foi baseada na proposta de [ConventionalCommits](https://www.conventionalcommits.org/pt-br/v1.0.0-beta.4/#resumo) com adaptações, de forma que a mensagem final de commits assume a seguinte forma:

* [<span style="color:#fc6a03">#NUMERO_ISSUE</span>] <span style="color:#00c5cd">tipo</span> - <span style="color:#e11584">breve descrição em inglês, com verbo no imperativo</span>

Sendo o <span style="color:#fc6a03">**Número da Issue**</span>  o Id atribuído a ela no momento de sua criação, o <span style="color:#00c5cd">**Tipo**</span> conforme descrito na próxima sessão e a <span style="color:#e11584">Breve Descrição</span> estando na língua inglesa, contendo o verbo principal no [imperativo](https://www.infoescola.com/ingles/frases-imperativas-imperatives/), conforme o seguinte exemplo:

* [<span style="color:#fc6a03">#15</span>] <span style="color:#00c5cd">feat</span> - <span style="color:#e11584">Add user validation to point info</span>


#### **Lista de Tipos**

Os tipos de commits a serem implementados são:

* ***test***

Indica qualquer tipo de criação ou alteração de códigos de teste. Exemplo: Criação de testes unitários.

```
[#1] test - add unit test of main module
```

* ***feat***

Indica o desenvolvimento de uma nova feature ao projeto. Exemplo: Acréscimo de um serviço, funcionalidade, endpoint, etc.

```
[#2] feat - create get location function 
```

* ***refactor*** 

Usado quando houver uma refatoração de código que não tenha qualquer tipo de impacto na lógica/regras de negócio do sistema. Exemplo: Mudanças de código após um code review

```
[#3] refactor - reorder validation flow
```

* ***style*** 

Empregado quando há mudanças de formatação e estilo do código que não alteram o sistema de nenhuma forma.
Exemplo: Mudar o style-guide, mudar de convenção lint, arrumar indentações, remover espaços em brancos, remover comentários, etc..

```
[#4] style - change primary color
```

* ***fix*** 

Utilizado quando há correção de erros que estão gerando bugs no sistema.
Exemplo: Aplicar tratativa para uma função que não está tendo o comportamento esperado e retornando erro.

```
[#5] fix - remove second callback
```

* ***chore*** 

Indica mudanças no projeto que não afetem o sistema ou arquivos de testes. São mudanças de desenvolvimento.
Exemplo: Mudar regras do eslint, adicionar prettier, adicionar mais extensões de arquivos ao .gitignore

```
[#6] chore - add map library
```

* ***docs*** 

Usado quando há mudanças na documentação do projeto.
Exemplo: adicionar informações na documentação da API, mudar o README, etc.

```
[#7] docs - add architecture document
```

* ***build*** 

Utilizada para indicar mudanças que afetam o processo de build do projeto ou dependências externas.
Exemplo: Gulp, adicionar/remover dependências do npm, etc.

```
[#8] build - remove unused dependencies
```

* ***perf*** 

Indica uma alteração que melhorou a performance do sistema.
Exemplo: alterar ForEach por while, melhorar a query ao banco, etc.

```
[#9] perf - optmize validation flow
```

* ***ci*** 

Utilizada para mudanças nos arquivos de configuração de CI.
Exemplo: Circle, Travis, BrowserStack, etc.

```
[#10] ci - implement ci/cd
```

* ***revert*** 

Indica a reverão de um commit anterior.

```
[#11] revert - revert previous commit
```

### [**Pull Requests**](#Sumário)

Deve seguir o mesmo padrão de nomenclatura da issue, e os usuŕios devem se atentar a linkar a respectiva issue a ser fechada pelo pull request.

#### **Nomenclatura**

A Nomenclatura de Pull Requests deve seguir o padrão:

```
[PREFIXO] Breve descrição do Pull Request em português
```

De forma que o elemento **PREFIXO** siga o seguinte padrão

| Prefixo | Tema | Exemplo |
| --- | --- | --- |
| DOCS | Documentação | `[DOCS] Melhorar README`|
| DEVOPS | Integração, DevOps | `[DEVOPS] Implementar CI/CD`|
| USXX | História de Usuário, em que *XX* se refera ao número da história de usuário | `[US12] Questionário para criação de uma nova comunidade`|
| IMPROVE | Melhoria ou adição de funcionalidades não correlacionadas a uma História de Usuário | `[IMPROVE] Adiciona regras de navegação ao frontend`|



