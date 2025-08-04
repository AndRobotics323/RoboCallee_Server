from django.urls import path
from django.conf.urls.static import static


# from pybo import views
from . import views

app_name = 'shoes' 

urlpatterns = [
    path('', views.index , name='index' ), 
    path('<int:shoe_id>/' , views.detail, name='detail'),
    path('try_on/' , views.try_on, name='try_on'),
    path('done_customer/' , views.done_customer, name='done_customer'),


    # path('answer/create/<int:question_id>/' , views.answer_create, name='answer_create'),
    # path('question/create/', views.question_create, name='question_create'),

]

