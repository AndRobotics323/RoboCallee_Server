from django.http import HttpResponse
from django.shortcuts import render, get_object_or_404, redirect
from django.utils import timezone



from robocallee_fms.srv import CustomerRequest 


import rclpy
from rclpy.node import Node


from .models import Product
# from .forms import QuestionForm, AnswerForm


from django.http import HttpResponseNotAllowed

from django.core.paginator import Paginator  
from django.contrib.auth.decorators import login_required

from django.db.models import Q


# Create your views here.

service_name = 'customer_service'

global_counter = 0



def index(request):
    global global_counter


    request.session.set_expiry(3600)  # 초 단위
 

    if 'customer_id' not in request.session:
        global_counter += 1
        request.session['customer_id'] = global_counter


    print(request.session['customer_id'] )

    shoes_list = Product.objects.all()


    kw = request.GET.get('kw', '')  # 검색어

    # print(kw)
    if kw:
        shoes_list = shoes_list.filter(
        Q(name__icontains=kw)   # 제목 검색
    ).distinct()

    # print(shoes_list)

    # shoes_list = Product.objects.order_by('-create_date')[:10]

    context = {'shoes_list': shoes_list, 'kw': kw}

    return render(request,  'shoes/shoes_list.html'  , context )
    #return HttpResponse("Hello, world! This is the Pybo index page.")



def detail(request, shoe_id):
    shoe = get_object_or_404(Product, pk=shoe_id)
    context = {'shoe': shoe}
    return render(request, 'shoes/shoes_detail.html', context)



def to_counter(request):

    return render(request, 'shoes/to_counter.html')


# rclpy 관련

rclpy.init(args=None)

node = rclpy.create_node('try_on_client')
client = node.create_client(CustomerRequest, service_name )


while not client.wait_for_service(timeout_sec=1.0):
    node.get_logger().info('Service not available, waiting again...')




def try_on(request):

    if request.method == 'POST':
        shoe_id = request.POST.get('shoe_id')
        shoe = get_object_or_404(Product, id=shoe_id)

        customer_id = request.session.get('customer_id')
        fitting_area = shoe.fitting_area

        req = CustomerRequest.Request()

        req.requester = "customer" 
        req.action = "come_here" 
        req.model = shoe.model
        req.size = shoe.size
        req.color = shoe.color
        req.x = float(shoe.x)
        req.y = float(shoe.y) 
        req.customer_id = int(customer_id)

        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future)

        if future.result() is not None:
            response = future.result()
            wait_list = response.wait_list

            return render(request, 'shoes/try_on.html', {'customer_id': customer_id, 'wait_list':wait_list, 'fitting_area' : fitting_area } )

        else:
            # node.destroy_node()
            # rclpy.shutdown()
            print("Service call failed" )


    return redirect('index')  # GET으로 접근한 경우



def done_customer(request):

    if request.method == 'POST':
        
        customer_id = request.session.get('customer_id')

        req = CustomerRequest.Request()

        req.requester = "customer" 
        req.action = "done" 
        req.model = "done"
        req.size = int(-1)
        req.color = "done"
        req.x = float(-1.0)
        req.y = float(-1.0)
        req.customer_id = int(customer_id)

        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future)


        
        if future.result() is not None:
            response = future.result()
            success = response.success
            if success:
                print('상품 도착 완료, customer_id : ' + str(customer_id) )

        else:
            # node.destroy_node()
            # rclpy.shutdown()
            print("Service call failed" )

            
            
    return redirect('index')  # 완료 or GET으로 접근한 경우




